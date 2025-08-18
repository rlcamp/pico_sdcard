#include "rp2350_ds3231.h"

#include "hardware/timer.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <time.h>

unsigned long long unix_microseconds_at_ref;
unsigned long long uptime_microseconds_at_ref;

static long long __tm_to_secs(const struct tm * tm) {
    /* adapted from musl libc, which is MIT licensed */
    long long year = tm->tm_year;
    int month = tm->tm_mon;
    if (month >= 12 || month < 0) {
        int adj = month / 12;
        month %= 12;
        if (month < 0) {
            adj--;
            month += 12;
        }
        year += adj;
    }

    int is_leap;
    long long t;

    if (year - 2ULL <= 136) {
        const int y = year;
        int leaps = (y - 68) >> 2;
        if (!((y - 68) & 3)) {
            leaps--;
            is_leap = 1;
        } else is_leap = 0;
        t = 31536000 * (y - 70) + 86400 * leaps;
    } else {
        int centuries, leaps;

        int cycles = (year - 100) / 400;
        int rem = (year - 100) % 400;
        if (rem < 0) {
            cycles--;
            rem += 400;
        }
        if (!rem) {
            is_leap = 1;
            centuries = 0;
            leaps = 0;
        } else {
            if (rem >= 200) {
                if (rem >= 300) centuries = 3, rem -= 300;
                else centuries = 2, rem -= 200;
            } else {
                if (rem >= 100) centuries = 1, rem -= 100;
                else centuries = 0;
            }
            if (!rem) {
                is_leap = 0;
                leaps = 0;
            } else {
                leaps = rem / 4U;
                rem %= 4U;
                is_leap = !rem;
            }
        }

        leaps += 97 * cycles + 24 * centuries - is_leap;

        t = (year - 100) * 31536000LL + leaps * 86400LL + 946684800 + 86400;
    }

    t += (0 == month ? 0 : 1 == month ? 31 : 2 == month ? 59 : 3 == month ? 90 :
          4 == month ? 120 : 5 == month ? 151 : 6 == month ? 181 : 7 == month ? 212 :
          8 == month ? 243 : 9 == month ? 273 : 10 == month ? 304 : 334) * 86400;
    if (is_leap && month >= 2) t += 86400;

    t += 86400LL * (tm->tm_mday - 1);
    t += 3600LL * tm->tm_hour;
    t += 60LL * tm->tm_min;
    t += tm->tm_sec;
    return t;
}

int ds3231_to_sys(void) {
    unsigned char buf[7];

    /* loop until a one-second boundary is seen, since we cannot count on pps yet */
    unsigned long long uptime_microseconds;
    unsigned sec_prior, sec = (unsigned)-1;
    do {
        sec_prior = sec;

        if (-1 == i2c_write_blocking(i2c0, 0x68, &(uint8_t){ 0 }, 1, true)) return -1;

        uptime_microseconds = timer_time_us_64(timer_hw);

        if (i2c_read_blocking(i2c0, 0x68, buf, 7, false) != 7) return -1;

        sec = 10 * ((buf[0] >> 4) & 0x7) + ((buf[0] >> 0) & 0xF);
    } while (sec != sec_prior);

    const unsigned min = 10 * ((buf[1] >> 4) & 0x7) + ((buf[1] >> 0) & 0xF);
    const unsigned hour = 10 * ((buf[2] >> 4) & 0x3) + ((buf[2] >> 0) & 0xF);
    const unsigned mday = 10 * ((buf[4] >> 4) & 0x3) + ((buf[4] >> 0) & 0xF);
    const unsigned mon = 10 * ((buf[5] >> 4) & 0x1) + ((buf[5] >> 0) & 0xF);
    const unsigned year = 10 * ((buf[6] >> 4) & 0xF) + ((buf[6] >> 0) & 0xF);

    dprintf(2, "%s: %04u%02u%02uT%02u%02u%02uZ\r\n", __func__,
            year + 2000, mon, mday, hour, min, sec);

    unix_microseconds_at_ref = __tm_to_secs(&(struct tm) {
        .tm_year = year + 2000 - 1900,
        .tm_mon = mon - 1, /* subtract one for struct tm months, which count from zero */
        .tm_mday = mday,
        .tm_hour = hour,
        .tm_min = min,
        .tm_sec = sec
    }) * 1000000ULL;
    uptime_microseconds_at_ref = uptime_microseconds;

    return 0;
}

