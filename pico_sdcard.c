/* proof of concept data logger to test sd card code */
#ifndef PROGNAME
#define PROGNAME "pico_sdcard"
#endif

/* lower level pico-sdk includes */
#include "hardware/xosc.h"
#include "hardware/structs/rosc.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"

/* so that we can reset into bootloader on command */
#include "pico/bootrom.h"

/* for SEV and WFE */
#include "RP2350.h"

/* other open source or project-agnostic includes by authors */
#include "cortex_m_cooperative_multitasking.h"
#include "rp2350_sdcard.h"
#include "cooperative_fatfs.h"
#include "rp2350_cooperative_uart.h"
#include "rp2350_cooperative_i2c.h"
#include "rp2350_ds3231.h"
#include "rp2350_tsys01.h"
#include "rp2350_kellerld.h"
#include "rp2350_ecezo.h"
#include "rp2350_bme280.h"

/* third party includes */
#include "ff.h"

/* c standard includes */
#include <stdio.h>
#include <stdlib.h>

volatile unsigned char verbose = 0;

extern void * sbrk(ptrdiff_t);

__attribute((noinline))
static int estimate_free_ram_from_main_thread(void) {
    /* this assumes it is being called in the main thread */
    char here;
    return (char *)&here - (char *)sbrk(0);
}

static size_t estimate_child_stack_usage(const size_t B, const unsigned char stack[restrict static B]) {
    for (size_t ib = 0; ib < B; ib++)
        if (stack[ib] != 0xFF) return B - ib;
    return 0;
}

static void set_first_value_in_string(char buf[], unsigned value) {
    char * cursor = buf;
    while (cursor[1] >= '0' && cursor[1] <= '9') cursor++;
    while (value && cursor >= buf) {
        *(cursor--) = '0' + (value % 10);
        value /= 10;
    }
    while (cursor >= buf)
        *(cursor--) = '0';
}

static int fputs_to_open_file(FIL * fp, const char * string) {
    const size_t len = strlen(string);
    FRESULT fres;
    UINT write_count;
    if ((fres = f_write(fp, string, len, &write_count))) {
        dprintf(2, "error: %s: f_write(): %d\r\n", __func__, fres);
        return -1;
    }
    if (write_count < len) {
        dprintf(2, "error: %s: short write\r\n", __func__);
        return -1;
    }
    return 0;
}

void run_from_xosc(void) {
    clock_configure_undivided(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, XOSC_MHZ * MHZ);
    clock_configure_undivided(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0, XOSC_MHZ * MHZ);

    clock_stop(clk_usb);
    clock_stop(clk_adc);
    clock_stop(clk_hstx);

    clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, XOSC_MHZ * MHZ);

    /* disable PLLs */
    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    /* disable rosc and wait for it to be stopped */
    rosc_hw->ctrl = (rosc_hw->ctrl & ~ROSC_CTRL_ENABLE_BITS) | (ROSC_CTRL_ENABLE_VALUE_DISABLE << ROSC_CTRL_ENABLE_LSB);
    while (rosc_hw->status & ROSC_STATUS_STABLE_BITS);
}

void lower_power_sleep_ms(const unsigned delay_ms) {
    /* get a timer, enable interrupt for alarm, but leave it disabled in nvic */
    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    /* arm timer */
    timer_hw->alarm[alarm_num] = timer_hw->timerawl + delay_ms * 1000;

    /* run other tasks or low power sleep until alarm interrupt */
    while (!(timer_hw->intr & (1U << alarm_num)))
        yield();

    /* acknowledge and clear the interrupt in both timer and nvic */
    hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
    irq_clear(hardware_alarm_get_irq_num(alarm_num));

    /* cleanup */
    hw_clear_bits(&timer_hw->inte, 1U << alarm_num);
    timer_hardware_alarm_unclaim(timer_hw, alarm_num);
}

/* allow an enable line to be shared between peripherals */
static volatile unsigned char enable_line_users = 0;

void enable_line_request(void) {
    /* cooperative lock is necessary so that we can yield within the init path */
    static volatile unsigned char locked = 0;
    while (locked) yield();
    locked = 1;

    if (!(enable_line_users++)) {
        gpio_init(22);
        gpio_set_dir(22, GPIO_OUT);
        gpio_put(22, 1);

        /* wait for inrush current, must also be > 1 ms for sdmmc */
        lower_power_sleep_ms(50);
    }

    locked = 0;
    __SEV();
}

void enable_line_release(void) {
    if (!(--enable_line_users)) {
        gpio_put(22, 0);
        gpio_deinit(22);
    }
}

/* length-two ring buffer populated by one task and consumed by zero or more others */
#define SAMPLE_RING_BUFFER_COUNT 4
_Static_assert(!(SAMPLE_RING_BUFFER_COUNT & (SAMPLE_RING_BUFFER_COUNT - 1)), "ring buffer must be a power of two");

static struct record {
    unsigned long long unix_microseconds;
    long temp_thousandths;
    long pressure_millibar;
    unsigned long conductivity_thousandths;
} records[SAMPLE_RING_BUFFER_COUNT];

static size_t irec_written = 0;

/* this gets incremented by anything that wants the sample data to be flowing. currently,
 tasks can only be started by the main thread, which will notice if this counter is nonzero
 and the sample task is not yet running, and start it. the task itself will notice if the
 counter has fallen to zero and shut down cleanly */
volatile unsigned char sample_consumers = 0;

static void sample_request(void) {
    /* if data was not already flowing... */
    if (!(sample_consumers++))
    /* allow the main thread to react to this becoming nonzero */
        __SEV();

    /* TODO: rework internals of cortex_m_cooperative_multitasking to allow tasks
     other than the main thread to start other tasks? */
}

static void sample_release(void) {
    if (!(--sample_consumers))
    /* allow the sample thread to react to this count dropping to zero */
        __SEV();
}

static void sample(void) {
    /* get a timer, enable interrupt for alarm, but leave it disabled in nvic */
    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    const unsigned period_microseconds = 1000000;

    /* first tick will be one interval from now */
    timer_hw->alarm[alarm_num] = timer_hw->timerawl + period_microseconds;

    /* need to request the first read >= 600 ms before we need it */
    ecezo_request_read();

    while (1) {
        /* run other tasks or low power sleep until next alarm interrupt */
        while (!(timer_hw->intr & (1U << alarm_num)))
            yield();

        const unsigned long long uptime_now = timer_time_us_64(timer_hw);

        /* acknowledge and clear the interrupt in both timer and nvic */
        hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
        irq_clear(hardware_alarm_get_irq_num(alarm_num));

        /* before rearming timer, check whether we should stop */
        if (!sample_consumers) break;

        /* increment and rearm the alarm */
        timer_hw->alarm[alarm_num] += period_microseconds;

        struct record * slot = records + irec_written % SAMPLE_RING_BUFFER_COUNT;
        memset(slot, 0, sizeof(struct record));

        slot->unix_microseconds = uptime_now + unix_microseconds_at_t0;

        /* read all the sensors. these block and internally call yield() a bunch, during
         which the other task can make progress on the previous card write if necessary */
        tsys01_read(&slot->temp_thousandths);
        kellerld_read(&slot->pressure_millibar, NULL);
        ecezo_finish_read(&slot->conductivity_thousandths);

        /* need to request that the next ecezo read be started because it takes 600 ms */
        ecezo_request_read();

        irec_written++;
        /* enforce that other threads see all of the above happen */
        __DSB();

        /* allow other threads to react */
        __SEV();
        yield();
    }

    /* deinit timer */
    hw_clear_bits(&timer_hw->inte, 1U << alarm_num);
    timer_hardware_alarm_unclaim(timer_hw, alarm_num);
}

volatile char stop_requested = 0;

int record(void) {
    stop_requested = 0;

    /* this is big, so don't put it on call stack */
    __attribute((aligned(4))) static FIL * fp = &(static FIL) { };
    FRESULT fres;

    /* loop until we get to the first available filename */
    unsigned file_id = 0;
    char path[] = "000000.csv";

    while (1) {
        set_first_value_in_string(path, file_id);
        fres = f_open(fp, path, FA_CREATE_NEW | FA_WRITE);
        if (FR_OK == fres)
            break;
        if (FR_EXIST == fres) {
            file_id++;
            continue;
        } else {
            dprintf(2, "%s: f_open(\"%s\"): %d\r\n", __func__, path, fres);
            return -1;
        }
    }

    /* let other tasks potentially do things with the card while we wait for data */
    card_unlock();

    dprintf(2, "%s: opened \"%s\"\r\n", __func__, path);

    /* wild guess of line format: time, temperature, ... */
    char line[] = "0123456789.000,+000.000,+0000.000,00000.000\n";

    /* subscribe to the data feed */
    size_t irec_read = *(volatile size_t *)&irec_written;

    while (1) {
        size_t irec_written_now;

        /* run other tasks or low power sleep until writer advances ring buffer */
        while (irec_read == (irec_written_now = *(volatile size_t *)&irec_written))
            yield();

        /* if we fell behind the writer, fast forward and emit a warning */
        if (irec_written_now - irec_read > SAMPLE_RING_BUFFER_COUNT - 1) {
            const size_t skipped = (irec_written_now - irec_read) - (SAMPLE_RING_BUFFER_COUNT - 1);
            dprintf(2, "warning: %s: missed %u records\r\n", __func__, (unsigned)skipped);
            irec_read += skipped;
        }

        /* if main thread requested that we stop, break out of logging loop and clean up */
        if (stop_requested) break;

        const struct record * slot = records + irec_read % SAMPLE_RING_BUFFER_COUNT;
        irec_read++;

        const unsigned long long now = slot->unix_microseconds;
        const unsigned long long now_ms = (now + 500ULL) / 1000ULL;
        const unsigned long long now_seconds_portion = now_ms / 1000ULL;
        const unsigned long long now_milliseconds_portion = now_ms % 1000ULL;

        set_first_value_in_string(line + 0, now_seconds_portion);
        set_first_value_in_string(line + 11, now_milliseconds_portion);

        const unsigned long abs_thousandths = labs(slot->temp_thousandths);
        const unsigned long a = abs_thousandths / 1000;
        const unsigned long b = abs_thousandths % 1000;

        set_first_value_in_string(line + 16, a);
        set_first_value_in_string(line + 20, b);
        line[15] = slot->temp_thousandths < 0 ? '-' : '+';

        const unsigned long abs_mbar = labs(slot->pressure_millibar);
        const unsigned long c = abs_mbar / 1000;
        const unsigned long d = abs_mbar % 1000;
        set_first_value_in_string(line + 25, c);
        set_first_value_in_string(line + 30, d);
        line[24] = slot->pressure_millibar < 0 ? '-' : '+';

        const unsigned long e = slot->conductivity_thousandths / 1000;
        const unsigned long f = slot->conductivity_thousandths % 1000;
        set_first_value_in_string(line + 34, e);
        set_first_value_in_string(line + 40, f);

        /* since we're doing cooperative multitasking, and the writer is another task
         rather than an interrupt or DMA, AND none of the above string manipulation calls
         yield(), we do NOT have to re-check here whether the writer lapped the reader
         during the above reads. if we did have to worry about that, we would have one last
         opportunity here to skip logging of this garbled line */

        /* reacquire the lock */
        card_lock();

        /* this will usually return immediately, occasionally it will internally yield() */
        if (-1 == fputs_to_open_file(fp, line)) return -1;

        card_unlock();

        dprintf(2, "%s", line);
    }

    card_lock();

    if ((fres = f_close(fp))) {
        dprintf(2, "%s: f_close(\"%s\"): %d\r\n", __func__, path, fres);
        return -1;
    }

    card_unlock();
    dprintf(2, "%s: closed \"%s\"\r\n", __func__, path);
    card_lock();

    return 0;
}

void record_outer(void) {
    if (-1 == card_request()) return;

    /* request that data start flowing if it is not already */
    sample_request();

    record();

    /* notify sample task that it can stop */
    sample_release();

    card_release();
}

static int bme280_read_and_print(void) {
    long temp_hundredths;
    unsigned long pressure_256ths, humidity_1024ths;
    if (-1 == bme280_read(&temp_hundredths, &pressure_256ths, &humidity_1024ths)) return -1;

    const unsigned long pressure_tenths = (pressure_256ths * 10ULL + 128) / 256;
    const unsigned long pressure_int = pressure_tenths / 10;
    const unsigned pressure_frac = pressure_tenths % 10;

    const unsigned long temp_abs_hundredths = labs(temp_hundredths);
    const long temp_int = (temp_abs_hundredths / 100) * (temp_hundredths < 0 ? -1 : 1);
    const unsigned temp_frac = temp_abs_hundredths % 100;

    const unsigned long humidity_thousandths = (humidity_1024ths * 1000UL + 512) / 1024;
    const unsigned long humidity_int = humidity_thousandths / 1000;
    const unsigned humidity_frac = humidity_thousandths % 1000;

    dprintf(2, "%s: %ld.%02u deg C, %lu.%01u Pa, %lu.%03u%% humidity\r\n", __func__,
            temp_int, temp_frac, pressure_int, pressure_frac, humidity_int, humidity_frac);

    return 0;
}

int main(void) {
    run_from_xosc();

    scb_hw->scr |= M33_SCR_SLEEPDEEP_BITS;

    /* turn off clocks to a bunch of stuff we aren't (yet) using */
    clocks_hw->wake_en1 = (CLOCKS_WAKE_EN1_BITS &
                           ~(CLOCKS_WAKE_EN1_CLK_USB_BITS|
                             CLOCKS_WAKE_EN1_CLK_SYS_USBCTRL_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_UART1_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_UART1_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_TRNG_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_SPI0_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_SPI0_BITS));

    clocks_hw->wake_en0 = (CLOCKS_WAKE_EN0_BITS &
                           ~(CLOCKS_WAKE_EN0_CLK_SYS_SHA256_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PWM_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PLL_USB_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PLL_SYS_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO2_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO1_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO0_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_JTAG_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_I2C0_BITS | /* will be reenabled on demand */
                             CLOCKS_WAKE_EN0_CLK_SYS_HSTX_BITS |
                             CLOCKS_WAKE_EN0_CLK_HSTX_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_ADC_BITS |
                             CLOCKS_WAKE_EN0_CLK_ADC_BITS));

    /* make sure we don't clock anything in sleep that wasn't clocked in wake */
    clocks_hw->sleep_en1 = clocks_hw->wake_en1;
    clocks_hw->sleep_en0 = clocks_hw->wake_en0;

    /* enable sevonpend so that we don't need a nearly empty isr */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    cooperative_uart_init();

    dprintf(2, "\r\n%s: built from %s\r\n", PROGNAME, GIT_VERSION);

    if (-1 == ds3231_to_sys())
        dprintf(2, "%s: could not read ds3231\r\n", PROGNAME);
    else
        dprintf(2, "%s: successfully read ds3231\r\n", PROGNAME);

    if (-1 == tsys01_init())
        dprintf(2, "%s: could not initialize tsys01\r\n", PROGNAME);
    else
        dprintf(2, "%s: successfully initted tsys01\r\n", PROGNAME);

    if (-1 == kellerld_init())
        dprintf(2, "%s: could not initialize kellerld\r\n", PROGNAME);
    else
        dprintf(2, "%s: successfully initted kellerld\r\n", PROGNAME);

    if (-1 == ecezo_init())
        dprintf(2, "%s: could not initialize ecezo\r\n", PROGNAME);
    else
        dprintf(2, "%s: successfully initted ecezo\r\n", PROGNAME);

    if (-1 == bme280_init())
        dprintf(2, "%s: could not initialize bme280\r\n", PROGNAME);
    else {
        dprintf(2, "%s: successfully initted bme280\r\n", PROGNAME);
        bme280_read_and_print();
    }

    static struct __attribute((aligned(8))) {
        /* this needs to be enough to accommodate the deepest call stack needed
         by a child task, PLUS any interrupt handlers if we are not using the
         msp/psp switch to provide interrupt handlers with their own dedicated
         call stack. this is probably still overkill */
        unsigned char stack[4096 - 16];

        struct child_context child;
    } child_sample, child_record;

    /* purely for diagnostic purposes */
    memset(child_sample.stack, 0xFF, sizeof(child_sample.stack));
    memset(child_record.stack, 0xFF, sizeof(child_record.stack));

//    child_start(&child_record.child, record_outer);

    /* loop on characters from uart */
    while (1) {
        const char * line = get_line_from_uart();

        /* if we got a complete line... */
        if (line) {
            const unsigned long long uptime_now = timer_time_us_64(timer_hw);
            dprintf(2, "%% %s\r\n", line);

            if (0 == gpzda_to_sys(line, 115200, uptime_now))
                dprintf(2, "%s: got valid timestamp\r\n", PROGNAME);

            else if (!strcmp(line, "start") && !child_is_running(&child_record.child))
                child_start(&child_record.child, record_outer);

            else if (!strcmp(line, "stop"))
                stop_requested = 1;

            else if (line == strstr(line, "ls "))
                ls(line + 3);
            else if (!strcmp(line, "ls"))
                ls(NULL);
            else if (line == strstr(line, "cat "))
                cat(line + 4);

            else if (line == strstr(line, "ecezo ") && !child_is_running(&child_sample.child))
                ecezo_command(line + 6);

            else if (!strcmp(line, "flash")) {
                dprintf(2, "%s: resetting into bootloader\r\n", PROGNAME);
                uart_tx_wait_blocking_with_yield();
                rom_reset_usb_boot_extra(-1, 0, false);
            }

            else if (!strcmp(line, "hctosys"))
                ds3231_to_sys();
            else if (!strcmp(line, "systohc"))
                sys_to_ds3231();
            else if (!strcmp(line, "bme280"))
                bme280_read_and_print();

            else if (!strcmp(line, "uptime"))
                dprintf(2, "%s: uptime %lu\r\n", PROGNAME, (unsigned long)(uptime_now / 1000000ULL));

            else if (line == strstr(line, "verbose "))
                verbose = strtoul(line + 8, NULL, 10);

            else if (!strcmp(line, "tasks"))
                dprintf(2, PROGNAME ": child tasks running: sample: %u, record: %u\r\n",
                        child_is_running(&child_record.child),
                        child_is_running(&child_sample.child));

            else if (!strcmp(line, "mem")) {
                extern unsigned char end[]; /* provided by linker script, used by sbrk */
                dprintf(2, "%s: sample child stack high water: %d bytes\r\n", PROGNAME,
                        estimate_child_stack_usage(sizeof(child_sample.stack), child_sample.stack));
                dprintf(2, "%s: record child stack high water: %d bytes\r\n", PROGNAME,
                        estimate_child_stack_usage(sizeof(child_record.stack), child_record.stack));
                dprintf(2, "%s: heap usage high water: %d bytes\r\n", PROGNAME, (uintptr_t)sbrk(0) - (uintptr_t)end);
                dprintf(2, "%s: free ram at least %d bytes\r\n", PROGNAME, estimate_free_ram_from_main_thread());
            }
        }

        /* if there is a nonzero number of consumers requesting that sample data be flowing
         and sample data is not yet flowing, start it TODO: factor this out */
        if (sample_consumers && !child_is_running(&child_sample.child))
            child_start(&child_sample.child, sample);

        yield();
    }
}
