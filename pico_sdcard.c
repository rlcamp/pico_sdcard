#include "pico/stdio_uart.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "RP2350.h"
#include "ff.h"
#include "rp2350_sdcard.h"

#include <stdio.h>
#include <string.h>

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

void yield(void) {
    __DSB(); __WFE();
}

__attribute((aligned(4))) static FATFS * fs = &(static FATFS) { };
__attribute((aligned(4))) static FIL * fp = &(static FIL) { };

int ls(void) {
    FRESULT fres;
    DIR dir;
    FILINFO info;

    if ((fres = f_opendir(&dir, "")) != FR_OK) {
        dprintf(2, "error: %s: f_opendir(): %d\r\n", __func__, fres);
        return -1;
    }

    while (FR_OK == f_readdir(&dir, &info) && info.fname[0] != '\0') {
        dprintf(2, "%s\r\n", info.fname);
        __SEV();
        yield();
    }

    f_closedir(&dir);
    return 0;
}

int main(void) {
//    set_sys_clock_48mhz();
    set_sys_clock_hz(96000000, true);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    /* enable sevonpend so that we don't need a nearly empty isr */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    stdio_uart_init();
    dprintf(2, "hello world stderr\r\n");

    do {
        FRESULT fres;
        if ((fres = f_mount(fs, "", 1))) {
            if (FR_NOT_READY == fres)
                dprintf(2, "error: %s: card apparently not present\r\n", __func__);
            else
                dprintf(2, "error: %s: f_mount(): %d\r\n", __func__, fres);
            break;
        }
        dprintf(2, "%s: mounted\r\n", __func__);
        if (-1 == ls()) break;

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
                break;
            }
        }

        if (fres != FR_OK) break;

        dprintf(2, "%s: opened \"%s\"\r\n", __func__, path);

        if (-1 == fputs_to_open_file(fp, "hello\n")) break;

        if ((fres = f_close(fp))) {
            dprintf(2, "%s: f_close(\"%s\"): %d\r\n", __func__, path, fres);
            break;
        }

        dprintf(2, "%s: done\r\n", __func__);

    } while(0);

    gpio_put(22, 0);

    while (1) yield();
    NVIC_SystemReset();
}
