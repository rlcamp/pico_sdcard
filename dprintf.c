/* this is a minimal from-the-ground-up implementation of dprintf which does no buffering
 at all, instead simply writing each individual byte or string segment to an fd. only a
 subset of the format strings implemented by newlib-nano are implemented here */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <unistd.h>

static_assert(sizeof(size_t) == sizeof(long), "not ilp32 or lp64");

/* we want to be able to call this directly from one other place */
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
void vfprintf_immediate(void (* write_func)(void *, const void *, const size_t),
                        void * cv, const char * restrict fmt, va_list ap) {
    for (; *fmt; fmt++)
        if (*fmt != '%') {
            const char * start = fmt;
            while (*(fmt + 1) && *(fmt + 1) != '%') fmt++;
            write_func(cv, start, fmt - start + 1);
        }
        else if ('%' == *(++fmt))
            write_func(cv, fmt, 1);
        else if ('s' == *fmt) {
            const char * string = va_arg(ap, char *);
            write_func(cv, string, strlen(string));
        } else {
            unsigned min_width = 1;
            if ('0' == *fmt)
                for (min_width = 0; (unsigned char)*(++fmt) - '0' < 10; )
                    min_width = min_width * 10 + (*fmt - '0');

            const char is_long = 'l' == *fmt, is_size = 'z' == *fmt;
            if (is_long || is_size) fmt++;

            if ('u' == *fmt || 'd' == *fmt || 'x' == *fmt || 'X' == *fmt) {
                unsigned long val;
                if ('d' == *fmt) {
                    const long sval = is_long ? va_arg(ap, long) : va_arg(ap, int);
                    if (sval < 0) {
                        write_func(cv, &(char) { '-' }, 1);
                        min_width--;
                        val = (sval != LONG_MIN) ? -sval : sval;
                    }
                    else val = sval;
                }
                else val = is_size ? va_arg(ap, size_t) : is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned);

                if ('x' == *fmt || 'X' == *fmt) {
                    unsigned shift = 4 * min_width;
                    for (; shift <= sizeof(unsigned long) * CHAR_BIT - 4 && (1UL << shift) - 1 < val; shift += 4);
                    for (; (shift = shift - 4) + 4 > 0;) {
                        const unsigned byte = (val >> shift) & 0xf;
                        write_func(cv, &(char){ byte <= 9 ? byte + '0' : (byte - 10) + 'A' }, 1);
                    }
                } else {
                    unsigned long div = 1;
                    for (unsigned w = 1; w < min_width || (div <= ULONG_MAX / 10 && div * 10 <= val); div *= 10, w++);
                    for (; div; div /= 10)
                        write_func(cv, &(char){ ((val / div) % 10) + '0' }, 1);
                }
            }
        }
}

static void write_wrapper(void * cv, const void * buf, const size_t size) {
    const int fd = *(int *)cv;
    write(fd, buf, size);
}

int dprintf(int fd, const char * restrict fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf_immediate(write_wrapper, &fd, fmt, ap);
    va_end(ap);

    return 0;
}
