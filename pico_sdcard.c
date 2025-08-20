#ifndef PROGNAME
#define PROGNAME "pico_sdcard"
#endif

/* proof of concept data logger to test sd card code */

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

/* need to be able to tell fatfs internals that it will have to reinit the card */
extern unsigned char diskio_initted;

unsigned char verbose = 1;

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

__attribute((aligned(4))) static FATFS * fs = &(static FATFS) { };
__attribute((aligned(4))) static FIL * fp = &(static FIL) { };

volatile char stop_requested = 0;

int record(void) {
    stop_requested = 0;

    FRESULT fres;
    if ((fres = f_mount(fs, "", 1))) {
        if (FR_NOT_READY == fres)
            dprintf(2, "error: %s: card apparently not present\r\n", __func__);
        else
            dprintf(2, "error: %s: f_mount(): %d\r\n", __func__, fres);
        return -1;
    }

    dprintf(2, "%s: mounted\r\n", __func__);

    /* loop until we get to the first available filename */
    unsigned file_id = 0;
    char path[] = "000000.txt";

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

    dprintf(2, "%s: opened \"%s\"\r\n", __func__, path);

    /* get a timer, enable interrupt for alarm, but leave it disabled in nvic */
    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    const unsigned period_microseconds = 1000000;

    /* first tick will be one interval from now */
    timer_hw->alarm[alarm_num] = timer_hw->timerawl + period_microseconds;

    /* need to request the first read >= 600 ms before we need it */
    ecezo_request_read();

    /* wild guess of line format: time, temperature, ... */
    char line[] = "0123456789.000,+000.000,+0000.000,+00000.000\n";

    for (size_t iline = 0;; iline++) {
        /* run other tasks or low power sleep until next alarm interrupt */
        while (!(timer_hw->intr & (1U << alarm_num)))
            yield();

        const unsigned long long uptime_now = timer_time_us_64(timer_hw);

        /* acknowledge and clear the interrupt in both timer and nvic */
        hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
        irq_clear(hardware_alarm_get_irq_num(alarm_num));

        /* before rearming timer, check whether we should stop */
        if (stop_requested) break;

        /* increment and rearm the alarm */
        timer_hw->alarm[alarm_num] += period_microseconds;

        /* set timestamp within output line */
        /* TODO: add actual time offset via ds3231 */
        const unsigned long long now = uptime_now - uptime_microseconds_at_ref + unix_microseconds_at_ref;
        const unsigned long long now_ms = (now + 500ULL) / 1000ULL;
        const unsigned long long now_seconds_portion = now_ms / 1000ULL;
        const unsigned long long now_milliseconds_portion = now_ms % 1000ULL;

        set_first_value_in_string(line + 0, now_seconds_portion);
        set_first_value_in_string(line + 11, now_milliseconds_portion);

        long temp_thousandths;
        if (tsys01_read(&temp_thousandths) != -1) {
            const unsigned long abs_thousandths = labs(temp_thousandths);
            const unsigned long a = abs_thousandths / 1000;
            const unsigned long b = abs_thousandths % 1000;

            set_first_value_in_string(line + 16, a);
            set_first_value_in_string(line + 20, b);
            line[15] = temp_thousandths < 0 ? '-' : '+';
        }

        long pressure_millibar;
        if (kellerld_read(&pressure_millibar, NULL) != -1) {
            const unsigned long abs_mbar = labs(pressure_millibar);
            const unsigned long c = abs_mbar / 1000;
            const unsigned long d = abs_mbar % 1000;
            set_first_value_in_string(line + 25, c);
            set_first_value_in_string(line + 30, d);
            line[24] = pressure_millibar < 0 ? '-' : '+';
        }

        long conductivity_thousandths;
        if (ecezo_finish_read(&conductivity_thousandths) != -1) {
            const unsigned long abs_cond = labs(conductivity_thousandths);
            const unsigned long a = abs_cond / 1000;
            const unsigned long b = abs_cond % 1000;
            set_first_value_in_string(line + 35, a);
            set_first_value_in_string(line + 41, b);
            line[34] = conductivity_thousandths < 0 ? '-' : '+';
        }

        /* this will usually return immediately, occasionally it will internally yield() */
        if (-1 == fputs_to_open_file(fp, line)) return -1;

        dprintf(2, "%s", line);

        /* need to request that the next ecezo read be started because it takes 600 ms */
        ecezo_request_read();
    }
    dprintf(2, "\r\n");

    /* deinit timer */
    hw_clear_bits(&timer_hw->inte, 1U << alarm_num);
    timer_hardware_alarm_unclaim(timer_hw, alarm_num);

    if ((fres = f_close(fp))) {
        dprintf(2, "%s: f_close(\"%s\"): %d\r\n", __func__, path, fres);
        return -1;
    }

    if ((fres = f_unmount(""))) {
        dprintf(2, "error: %s: f_unmount(): %d\r\n", __func__, fres);
        return -1;
    }

    dprintf(2, "%s: done\r\n", __func__);
    return 0;
}

void record_outer(void) {
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    lower_power_sleep_ms(10);

    record();

    /* make sure we will reinit when we get here next */
    diskio_initted = 0;

    gpio_put(22, 0);
    gpio_deinit(22);
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

    dprintf(2, "\r\nhello\r\n");

    if (-1 == ds3231_to_sys())
        dprintf(2, "%s: could not read ds3231\r\n", PROGNAME);
    else
        dprintf(2, "%s: successfully read ds3231\r\n", PROGNAME);

    if (-1 == tsys01_init())
        dprintf(2, "%s: could not initialize tsys01\r\n", PROGNAME);

    if (-1 == kellerld_init())
        dprintf(2, "%s: could not initialize kellerld\r\n", PROGNAME);

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
    } child_record;

    /* purely for diagnostic purposes */
    memset(child_record.stack, 0xFF, sizeof(child_record.stack));

    child_start(&child_record.child, record_outer);

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

            else if (line == strstr(line, "ecezo ") && !child_is_running(&child_record.child))
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

            else if (!strcmp(line, "mem")) {
                extern unsigned char end[]; /* provided by linker script, used by sbrk */
                dprintf(2, "%s: record child stack high water: %d bytes\r\n", PROGNAME,
                        estimate_child_stack_usage(sizeof(child_record.stack), child_record.stack));
                dprintf(2, "%s: heap usage high water: %d bytes\r\n", PROGNAME, (uintptr_t)sbrk(0) - (uintptr_t)end);
                dprintf(2, "%s: free ram at least %d bytes\r\n", PROGNAME, estimate_free_ram_from_main_thread());
            }
        }
        yield();
    }
}
