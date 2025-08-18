#include "rp2350_cooperative_i2c.h"
#include "hardware/i2c.h"

#include <math.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

extern void lower_power_sleep_ms(unsigned);

static uint16_t coefficients[5] = { };

int tsys01_init(void) {
    i2c_request();
    if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t){ 0x1E }, 1, false)) {
        i2c_release();
        return -1;
    }

    i2c_unlock();
    lower_power_sleep_ms(10);
    i2c_lock();

    for (size_t iprom = 0; iprom < 5; iprom++) {
        if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t){ 0xA2 + 2 * iprom }, 1, false)) {
            i2c_release();
            return -1;
        }

        uint16_t tmp;
        if (-1 == i2c_read_blocking(i2c0, 0x77, (void *)&tmp, 2, false)) {
            i2c_release();
            return -1;
        }
        coefficients[4 - iprom] = __builtin_bswap16(tmp);
    }

    i2c_release();
    return 0;
}

int tsys01_read_thousandths(void) {
    i2c_request();
    if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t){ 0x48 }, 1, false)) {
        i2c_release();
        return INT_MIN;
    }

    i2c_unlock();
    lower_power_sleep_ms(10);
    i2c_lock();

    if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t){ 0x00 }, 1, false)) {
        i2c_release();
        return INT_MIN;
    }

    unsigned char bytes[3];
    if (-1 == i2c_read_blocking(i2c0, 0x77, bytes, 3, false)) {
        i2c_release();
        return INT_MIN;
    }

    i2c_release();

    const uint32_t adc24 = (bytes[0] << 16) | (bytes[1] << 8) | bytes[2];

    /* TODO: this is literal from the datasheet, if possible rework all this sketchy math
     to return temperature in thousandths of a degree without doing any floating point */
    const float adc16 = adc24 / 256.0f;
    const float adc16_2 = adc16 * adc16;
    const float adc16_3 = adc16_2 * adc16;
    const float adc16_4 = adc16_2 * adc16_2;

    const float temp = (-2.0f * coefficients[4] * 1e-21f * adc16_4 +
                        +4.0f * coefficients[3] * 1e-16f * adc16_3 +
                        -2.0f * coefficients[2] * 1e-11f * adc16_2 +
                        +1.0f * coefficients[1] *  1e-6f * adc16 +
                        -1.5f * coefficients[0] *  1e-2f);
    return lrintf(temp * 1e3f);
}

