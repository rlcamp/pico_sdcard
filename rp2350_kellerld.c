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

static int read_three_bytes(unsigned char bytes[3], unsigned char addr) {
    if (-1 == i2c_write_blocking(i2c0, 0x40, &addr, 1, false)) return -1;

    i2c_unlock();
    lower_power_sleep_ms(2);
    i2c_lock();

    if (-1 == i2c_read_blocking(i2c0, 0x40, bytes, 3, false)) return -1;

    return 0;
}

static float pressure_min, pressure_max, pressure_mode;

int kellerld_init(void) {
    /* initialize the i2c bus and acquire a lock */
    i2c_request();

    unsigned char b[6];
    if (-1 == read_three_bytes(b, 0x12)) {
        i2c_release();
        return -1;
    }

    const uint16_t scaling0 = (b[1] << 8) | b[2];
    const unsigned mode = scaling0 & 0b11;
    const unsigned year = scaling0 >> 11;
    const unsigned month = (scaling0 & 0b11110000000) >> 7;
    const unsigned day = (scaling0 & 0b1111100) >> 2;

    dprintf(2, "%s: scaling0 mode = %u, year %u, month %u, day %u\r\n", __func__, mode, year, month, day);

    pressure_mode = 0 == mode ? 1.03125f : 1 == mode ? 1.0f : 0.0f;

    if (-1 == read_three_bytes(b + 0, 0x13) ||
        -1 == read_three_bytes(b + 3, 0x14)) {
        i2c_release();
        return -1;
    }

    memcpy(&pressure_min, &(uint32_t) { (b[1] << 24) | (b[2] << 16) | (b[4] << 8) | b[5] }, sizeof(uint32_t));

    if (-1 == read_three_bytes(b + 0, 0x15) ||
        -1 == read_three_bytes(b + 3, 0x16)) {
        i2c_release();
        return -1;
    }

    memcpy(&pressure_max, &(uint32_t) { (b[1] << 24) | (b[2] << 16) | (b[4] << 8) | b[5] }, sizeof(uint32_t));

    /* done with i2c bus */
    i2c_release();
    return 0;
}

int kellerld_read(long * pressure_millibar_p, long * temperature_thousandths_p) {
    i2c_request();
    if (-1 == i2c_write_blocking(i2c0, 0x40, &(uint8_t){ 0xAC }, 1, false)) {
        i2c_release();
        return -1;
    }

    i2c_unlock();
    lower_power_sleep_ms(9);
    i2c_lock();

    unsigned char b[5];
    if (-1 == i2c_read_blocking(i2c0, 0x40, b, 5, false)) {
        i2c_release();
        return -1;
    }

    i2c_release();

    const uint16_t p_raw = (b[1] << 8) | b[2];
    const uint16_t t_raw = (b[3] << 8) | b[4];

    const float pressure_bar = ((float)p_raw - 16384.0f) * (pressure_max - pressure_min) / 32768.0f + pressure_min + pressure_mode;

    const float temperature_celsius = ((t_raw / 16U) - 24) * 0.05f - 50.0f;

    if (pressure_millibar_p)
        *pressure_millibar_p = lrintf(pressure_bar * 1000.0f);
    if (temperature_thousandths_p)
        *temperature_thousandths_p = lrintf(temperature_celsius * 1000.0f);

    return 0;
}
