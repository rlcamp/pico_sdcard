#include "rp2350_ds3231.h"
#include "rp2350_cooperative_i2c.h"

#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include <stdio.h>
#include <time.h>
#include <string.h>

extern void yield(void);

unsigned long long unix_microseconds_at_ref;
unsigned long long uptime_microseconds_at_ref;

static unsigned char byte_hex(unsigned char x) {
    return x >= '0' && x <= '9' ? x - '0' : (x >= 'A' && x <= 'Z' ? x - 'A' : x - 'a') + 10;
}

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
    i2c_request();
    unsigned char buf[7];

    /* loop until a one-second boundary is seen, since we cannot count on pps yet */
    unsigned long long uptime_microseconds;
    unsigned sec_prior, sec = (unsigned)-1;
    do {
        sec_prior = sec;

        if (-1 == i2c_write_blocking(i2c0, 0x68, &(uint8_t){ 0 }, 1, true)) {
            i2c_release();
            return -1;
        }

        uptime_microseconds = timer_time_us_64(timer_hw);

        if (i2c_read_blocking(i2c0, 0x68, buf, 7, false) != 7) {
            i2c_release();
            return -1;
        }

        sec = 10 * ((buf[0] >> 4) & 0x7) + ((buf[0] >> 0) & 0xF);
        __sev();
        yield();
    } while (sec != sec_prior);
    i2c_release();

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

unsigned long long wait_until_one_second_boundary(void) {
    const unsigned long long uptime_now = timer_time_us_64(timer_hw);
    const unsigned long long unix_microseconds_now = uptime_now - uptime_microseconds_at_ref + unix_microseconds_at_ref;
    const unsigned long long unix_microseconds_at_future_one_second_boundary = ((unix_microseconds_now + 1100000ULL) / 1000000ULL) * 1000000ULL;
    const unsigned long long uptime_at_future_one_second_boundary = unix_microseconds_at_future_one_second_boundary - unix_microseconds_at_ref + uptime_microseconds_at_ref;

    /* get a timer, enable interrupt for alarm, but leave it disabled in nvic */
    const unsigned alarm_num = timer_hardware_alarm_claim_unused(timer_hw, true);
    hw_set_bits(&timer_hw->inte, 1U << alarm_num);
    irq_set_enabled(hardware_alarm_get_irq_num(alarm_num), false);

    /* arm timer */
    timer_hw->alarm[alarm_num] = (uptime_at_future_one_second_boundary % 0xFFFFFFFF);

    /* run other tasks or low power sleep until alarm interrupt */
    while (!(timer_hw->intr & (1U << alarm_num)))
        yield();

    /* acknowledge and clear the interrupt in both timer and nvic */
    hw_clear_bits(&timer_hw->intr, 1U << alarm_num);
    irq_clear(hardware_alarm_get_irq_num(alarm_num));

    /* cleanup */
    timer_hardware_alarm_unclaim(timer_hw, alarm_num);

    return unix_microseconds_at_future_one_second_boundary;
}

int sys_to_ds3231(void) {
    i2c_request();
    i2c_unlock();

    unsigned long long unix_microseconds_now;
    do unix_microseconds_now = wait_until_one_second_boundary();
    while (-1 == i2c_lock_or_fail());

    const unsigned long long unix_microseconds_again = timer_time_us_64(timer_hw) - uptime_microseconds_at_ref + unix_microseconds_at_ref;

    /* when we return from the above, we have the i2c lock, and we know what time it is */

    struct tm tm;
    if (!gmtime_r(&(time_t) { unix_microseconds_now / 1000000ULL }, &tm)) return 0;
    const unsigned mon = tm.tm_mon + 1, year = tm.tm_year + 1900;

    /* the ideal time to start this transaction would be such that the seconds register
     finishes being written at exactly the indicated time, which is something like 27 baud
     periods into the message */

    if (-1 == i2c_write_blocking(i2c0, 0x68, (const unsigned char[]) {
        0,
        ((tm.tm_sec / 10U) << 4) | (tm.tm_sec % 10U),
        ((tm.tm_min / 10U) << 4) | (tm.tm_min % 10U),
        ((tm.tm_hour / 10U) << 4) | (tm.tm_hour % 10U),
        0,
        ((tm.tm_mday / 10U) << 4) | (tm.tm_mday % 10U),
        (((year - 2000) / 100U) << 7) | ((mon / 10U) << 4) | (mon % 10U),
        ((((year - 2000) % 100U) / 10U) << 4) | ((year - 2000) % 10U)
    }, 8, false)) {
        i2c_release();
        return -1;
    }

    i2c_release();

    dprintf(2, "%s: %lu\r\n", __func__, (unsigned long)(unix_microseconds_again - unix_microseconds_now));
    return 0;
}

int gpzda_to_sys(const char * line, const unsigned baud_rate, const unsigned long long uptime_microseconds_at_end_of_line) {
    if (line[0] != '$') return -1;

    unsigned int cksum = 0, hour = 0, min = 0, sec = 0, month = 0, day = 0, year = 0;
    unsigned frac_num = 0, frac_den = 1;

    const char * token = line;

    if ('Z' == line[3] && 'D' == line[4] && 'A' == line[5]) {
        for (unsigned itok = 0; ; itok++) {
            const size_t toklen = strcspn(token, ",*");

            if (1 == itok) {
                hour = (token[0] - '0') * 10U + (token[1] - '0');
                min = (token[2] - '0') * 10U + (token[3] - '0');
                sec = (token[4] - '0') * 10U + (token[5] - '0');
                if (token[6] == '.') {
                    const char * c = token + 7;
                    while (*c >= '0' && *c <= '9') {
                        frac_num = frac_num * 10 + (*c - '0');
                        frac_den *= 10;
                        c++;
                    }
                }
            }
            else if (2 == itok) day = (token[0] - '0') * 10U + (token[1] - '0');
            else if (3 == itok) month = (token[0] - '0') * 10U + (token[1] - '0');
            else if (4 == itok) year = ((token[0] - '0') * 1000U +
                                        (token[1] - '0') * 100U +
                                        (token[2] - '0') * 10U +
                                        (token[3] - '0'));
            else if (7 == itok) cksum = byte_hex(token[0]) * 16U + byte_hex(token[1]);

            if (token[toklen] == '\0') break;
            token += toklen + 1;
        }
    }
    else if ('R' == line[3] && 'M' == line[4] && 'C' == line[5]) {
        for (unsigned itok = 0; ; itok++) {
            const size_t toklen = strcspn(token, ",*");

            if (1 == itok) {
                hour = (token[0] - '0') * 10U + (token[1] - '0');
                min = (token[2] - '0') * 10U + (token[3] - '0');
                sec = (token[4] - '0') * 10U + (token[5] - '0');
                if (token[6] == '.') {
                    const char * c = token + 7;
                    while (*c >= '0' && *c <= '9') {
                        frac_num = frac_num * 10 + (*c - '0');
                        frac_den *= 10;
                        c++;
                    }
                }
            }
            else if (9 == itok) {
                day = (token[0] - '0') * 10U + (token[1] - '0');
                month = (token[2] - '0') * 10U + (token[3] - '0');
                year = (token[4] - '0') * 10U + (token[5] - '0') + 2000;
            }
            else if (13 == itok) cksum = byte_hex(token[0]) * 16U + byte_hex(token[1]);

            if (token[toklen] == '\0') break;
            token += toklen + 1;
        }
    }

    unsigned int cksum_calculated = 0;
    for (const unsigned char * c = (const unsigned char *)line + 1; *c != '\0' && *c != '*'; c++)
        cksum_calculated ^= *c;

    if (cksum != cksum_calculated) return -1;

    const long long out = __tm_to_secs(&(struct tm) {
        .tm_year = year - 1900,
        .tm_mon = month - 1, /* subtract one for struct tm months, which count from zero */
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = min,
        .tm_sec = sec
    });

    /* encoded time in 32768 Hz ticks */
    const unsigned long long unix_microseconds_encoded = out * 1000000ULL + (frac_num * 1000000ULL + frac_den / 2U) / frac_den;

    /* length in bytes of the nmea string encoding this time, including \r\n */
    const size_t line_length = token - line + 2;

    /* assumed duration of the nmea string as transmitted */
    const unsigned long long correction = (line_length * 10U * 1000000ULL + baud_rate / 2U) / baud_rate;

    uptime_microseconds_at_ref = uptime_microseconds_at_end_of_line;
    unix_microseconds_at_ref = unix_microseconds_encoded + correction;

    if (-1 == sys_to_ds3231())
        dprintf(2, "%s: failed to set ds3231 from sys\r\n", __func__);

    return 0;
}

uint32_t get_fattime(void) {
    const unsigned long long unix_microseconds = timer_time_us_64(timer_hw) - uptime_microseconds_at_ref + unix_microseconds_at_ref;

    struct tm out;
    if (!gmtime_r(&(time_t) { unix_microseconds / 1000000ULL }, &out)) return 0;

    return ((out.tm_year - 80) << 25U |
            (out.tm_mon + 1) << 21U |
            out.tm_mday << 16U |
            out.tm_hour << 11U |
            out.tm_min << 5U |
            out.tm_sec >> 1U);
}
