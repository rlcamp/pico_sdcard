#include "rp2350_bme280.h"
#include "rp2350_cooperative_i2c.h"
#include "hardware/i2c.h"

#include <stdint.h>

extern void lower_power_sleep_ms(unsigned);

static uint16_t T1;
static int16_t T2, T3;

static uint16_t P1;
static int16_t P2, P3, P4, P5, P6, P7, P8, P9;

static uint8_t H1;
static int16_t H2;
static uint8_t H3;
static int16_t H4, H5; /* crap */
static int8_t H6;

static int32_t t_fine;

int bme280_init(void) {
    /* initialize the i2c bus and acquire a lock */
    i2c_request();

    do {
        unsigned char bytes[26 + 7];
        if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t) { 0x88 }, 1, true) ||
            -1 == i2c_read_blocking(i2c0, 0x77, bytes + 0, 26, false) ||
            -1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t) { 0xE1 }, 1, true) ||
            -1 == i2c_read_blocking(i2c0, 0x77, bytes + 26, 7, false)) break;

        T1 = (((unsigned)bytes[1]) << 8) | (unsigned)bytes[0];
        T2 = (((unsigned)bytes[3]) << 8) | (unsigned)bytes[2];
        T3 = (((unsigned)bytes[5]) << 8) | (unsigned)bytes[4];

        P1 = (((unsigned)bytes[7]) << 8) | (unsigned)bytes[6];
        P2 = (((unsigned)bytes[9]) << 8) | (unsigned)bytes[8];
        P3 = (((unsigned)bytes[11]) << 8) | (unsigned)bytes[10];
        P4 = (((unsigned)bytes[13]) << 8) | (unsigned)bytes[12];
        P5 = (((unsigned)bytes[15]) << 8) | (unsigned)bytes[14];
        P6 = (((unsigned)bytes[17]) << 8) | (unsigned)bytes[16];
        P7 = (((unsigned)bytes[19]) << 8) | (unsigned)bytes[18];
        P8 = (((unsigned)bytes[21]) << 8) | (unsigned)bytes[20];
        P9 = (((unsigned)bytes[23]) << 8) | (unsigned)bytes[22];

        H1 = bytes[25];

        H2 = (((unsigned)bytes[27]) << 8) | (unsigned)bytes[26];
        H3 = bytes[28];
        H4 = (((unsigned)bytes[29]) << 4) | ((unsigned)bytes[30] & 0x0F);
        H5 = (((unsigned)bytes[31]) << 4) | ((unsigned)bytes[30] >> 4);
        H6 = bytes[32];

        /* set humidity osr to 1x */
        if (-1 == i2c_write_blocking(i2c0, 0x77, (uint8_t[]){ 0xF2, 0b1 }, 2, false)) break;

        /* done with i2c bus */
        i2c_release();

        return 0;

    } while(0);

    i2c_release();
    return -1;
}

int bme280_read(long * T_hundredths_p, unsigned long * press_256ths_p, unsigned long * humidity_1024ths_p) {
    i2c_request();

    do {
        /* set the pressure and temperature osrs to 1x and initiate forced mode */
        if (-1 == i2c_write_blocking(i2c0, 0x77, (uint8_t[]){ 0xF4, 0b100101 }, 2, false)) break;

        i2c_unlock();
        lower_power_sleep_ms(12); /* TODO: determine minimum time */
        i2c_lock();

        /* read status */
        unsigned char bytes[8];
        if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t){ 0xF3 }, 1, true) ||
            -1 == i2c_read_blocking(i2c0, 0x77, bytes, 1, false)) break;

        /* read adc values */
        if (-1 == i2c_write_blocking(i2c0, 0x77, &(uint8_t){ 0xF7 }, 1, true) ||
            -1 == i2c_read_blocking(i2c0, 0x77, bytes, 8, false)) break;

        i2c_release();

        const uint32_t adc_P = (bytes[0] << 12) | (bytes[1] << 4) | (bytes[2] >> 4);
        const uint32_t adc_T = (bytes[3] << 12) | (bytes[4] << 4) | (bytes[5] >> 4);
        const uint16_t adc_H = (bytes[6] << 8) | bytes[7];

        /* from datasheet: Returns temperature in DegC, resolution is 0.01 DegC. Output
         value of "5123" equals 51.23 DegC. */
        {
            /* verbatim from the datasheet, with whitespace and typedef mods */
            int32_t var1, var2;
            var1 = ((((adc_T >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
            var2 = (((((adc_T >> 4) - ((int32_t)T1)) * ((adc_T >> 4) - ((int32_t)T1))) >> 12) * ((int32_t)T3)) >> 14;
            t_fine = var1 + var2;
            *T_hundredths_p = (t_fine * 5 + 128) >> 8;
        }

        /* from datasheet: Returns pressure in Pa as unsigned 32 bit integer in Q24.8
         format (24 integer bits and 8 fractional bits). Output value of "24674867"
         represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa */
        if (press_256ths_p) {
            /* verbatim from the datasheet, with whitespace and control flow modded */
            int64_t var1, var2;
            var1 = (int64_t)t_fine - 128000;
            var2 = var1 * var1 * P6;
            var2 = var2 + ((var1 * P5) << 17);
            var2 = var2 + ((int64_t)P4 << 35);
            var1 = ((var1 * var1 * P3) >> 8) + ((var1 * P2) << 12);
            var1 = (((1LL << 47) + var1) * P1) >> 33;

            if (var1 != 0) {
                int64_t p;
                p = 1048576 - adc_P;
                p = (((p << 31) - var2) * 3125) / var1;
                var1 = (P9 * (p >> 13) * (p >> 13)) >> 25;
                var2 = (P8 * p) >> 19;

                *press_256ths_p = ((p + var1 + var2) >> 8) + (((int64_t)P7) << 4);
            }
            else
                *press_256ths_p = 0;
        }

        /* from datasheet: Returns humidity in %RH as unsigned 32 bit integer in Q22.10
         format (22 integer and 10 fractional bits). Output value of "47445" represents
         47445 / 1024 = 46.333 %RH */
        if (humidity_1024ths_p) {
            /* verbatim from the datasheet, with whitespace and typedef mods */
            int32_t v_x1_u32r;
            v_x1_u32r = t_fine - 76800;
            v_x1_u32r = (((((adc_H << 14) - (((int32_t)H4) << 20) - (((int32_t)H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)H6)) >> 10) * (((v_x1_u32r * ((int32_t)H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)H2) + 8192) >> 14));
            v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)H1)) >> 4));
            v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
            v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
            *humidity_1024ths_p = v_x1_u32r >> 12;
        }

        return 0;
    } while(0);

    i2c_release();
    return -1;
}
