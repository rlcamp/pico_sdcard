#include "rp2350_cooperative_fatfs.h"
#include "RP2350.h"

#include "hardware/gpio.h"

#include <stdio.h>
#include <unistd.h>

/* need to be able to tell fatfs internals that it will have to reinit the card */
extern unsigned char diskio_initted;

extern void yield(void);
extern void lower_power_sleep_ms(unsigned);

__attribute((aligned(4))) FATFS * fs = &(static FATFS) { };

static volatile unsigned char card_locked = 0, card_users = 0;

void card_lock(void) {
    while (card_locked) yield();
    card_locked = 1;
}

void card_unlock(void) {
    card_locked = 0;
    __SEV();
}

int card_request(void) {
    card_lock();

    if (!(card_users++)) {
        /* enable power to card */
        gpio_init(22);
        gpio_set_dir(22, GPIO_OUT);
        gpio_put(22, 1);

        lower_power_sleep_ms(10);

        /* we have a guarantee that when the above returns, it has been at least 1 ms since
         power was applied to the sd card */

        dprintf(2, "%s: mounting\r\n", __func__);

        FRESULT fres;
        if ((fres = f_mount(fs, "", 1))) {
            card_unlock();
            if (FR_NOT_READY == fres)
                dprintf(2, "error: %s: card apparently not present\r\n", __func__);
            else
                dprintf(2, "error: %s: f_mount(): %d\r\n", __func__, fres);
            return -1;
        }
    }

    /* caller holds the lock when this returns successfully */
    return 0;
}

void card_release(void) {
    /* caller is expected to still hold the lock */
    if (!(--card_users)) {
        /* fatfs doesn't give us any API to have it tell the lower level diskio code that
         the card has been power cycled and will have to be initted when mounting again */
        diskio_initted = 0;

        gpio_put(22, 0);
        gpio_deinit(22);
    }

    card_unlock();
}

int ls(void) {
    if (-1 == card_request()) return -1;

    FRESULT fres;
    static DIR dir;
    if ((fres = f_opendir(&dir, "")) != FR_OK) {
        card_release();
        dprintf(2, "error: %s: f_opendir(): %d\r\n", __func__, fres);
        return -1;
    }

    static FILINFO info;
    while (FR_OK == f_readdir(&dir, &info) && info.fname[0] != '\0') {
        if ('.' == info.fname[0]) continue;

        card_unlock();
        dprintf(2, "%s\r\n", info.fname);
        card_lock();

        /* give other tasks a chance to do something */
        __SEV();
        yield();
    }

    f_closedir(&dir);

    card_release();

    return 0;
}

int cat(const char * path) {
    if (-1 == card_request()) return -1;

    do {
        /* this is big, so don't put it on call stack */
        __attribute((aligned(4))) static FIL * fp = &(static FIL) { };
        FRESULT fres;

        if ((fres = f_open(fp, path, FA_OPEN_EXISTING | FA_READ))) {
            if (FR_NO_FILE == fres)
                dprintf(2, "%s: f_open(\"%s\"): no such file\r\n", __func__, path);
            else
                dprintf(2, "%s: f_open(\"%s\"): %d\r\n", __func__, path, fres);
            break;
        }

        /* read from file and write to uart in chunks until eof */
        UINT bytes_read;
        do {
            unsigned char buf[128];
            if ((fres = f_read(fp, buf, sizeof(buf), &bytes_read))) {
                dprintf(2, "error: %s: f_read(): %d\r\n", __func__, fres);
                break;
            }

            /* write to uart with card unlocked */
            card_unlock();
            write(2, buf, bytes_read);
            card_lock();

            /* let other tasks do some work */
            __SEV();
            yield();
        } while (bytes_read && !fres);

        if (fres) break;

        /* make sure we emit a newline to unlock the uart */
        dprintf(2, "\r\n");

        if ((fres = f_close(fp))) {
            dprintf(2, "%s: f_close(\"%s\"): %d\r\n", __func__, path, fres);
            break;
        }

        card_release();
        return 0;

    } while(0);

    card_release();
    return -1;
}
