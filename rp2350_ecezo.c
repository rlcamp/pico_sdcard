#include "rp2350_kellerld.h"
#include "rp2350_cooperative_i2c.h"
#include "hardware/i2c.h"

#include <math.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern void lower_power_sleep_ms(unsigned);

static int get_response_string(char buf[32]) {
    char * cur = buf;
    for (; cur < buf + 32; cur++) {
        if (-1 == i2c_read_burst_blocking(i2c0, 0x64, (void *)cur, 1))
            return -1;

        if (*cur == '\0') break;
    }

    i2c_read_blocking(i2c0, 0x64, NULL, 0, false);

    return '\0' == *cur ? 0 : -1;
}

int ecezo_init(void) {
    /* initialize the i2c bus and acquire a lock */
    i2c_request();

    do {
        if (-1 == i2c_write_blocking(i2c0, 0x64, &(uint8_t) { 'i' }, 1, false)) break;

        i2c_unlock();
        lower_power_sleep_ms(300);
        i2c_lock();

        char buf[32];
        if (-1 == get_response_string(buf)) break;

        dprintf(2, "%s: %s\r\n", __func__, buf + 1);

        /* done with i2c bus */
        i2c_release();
        return 0;

    } while(0);

    i2c_release();
    return -1;
}

int ecezo_request_read(void) {
    /* initialize the i2c bus and acquire a lock */
    i2c_request();

    if (-1 == i2c_write_blocking(i2c0, 0x64, &(uint8_t) { 'R' }, 1, false)) {
        i2c_release();
        return -1;
    }

    i2c_release();
    return 0;
}

int ecezo_finish_read(long * conductivity_thousandths_p) {
    /* initialize the i2c bus and acquire a lock */
    i2c_request();

    char buf[32];
    if (-1 == get_response_string(buf)) {
        i2c_release();
        return -1;
    }

    i2c_release();

    const size_t sizeof_conductivity = strcspn(buf + 1, ",\r\n");
    buf[sizeof_conductivity + 1] = '\0';

    const float conductivity = strtof(buf + 1, NULL);

    *conductivity_thousandths_p = lrintf(conductivity);

    return 0;
}
