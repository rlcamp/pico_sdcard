#include "rp2350_cooperative_i2c.h"
#include "RP2350.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

extern void yield(void);

/* intentionally cooperative only mutex-like thing, can be made actually thread safe but no need */
static volatile unsigned char lock = 0;
static volatile unsigned char users = 0;

void i2c_lock(void) {
    while (lock) yield();
    lock = 1;
}

int i2c_lock_or_fail(void) {
    if (lock) return -1;
    lock = 1;
    return 0;
}

void i2c_unlock(void) {
    lock = 0;

    /* inhibit the next wfe call, to make sure other threads get a chance to react to the
     lock being released if they were waiting for it, before the processor sleeps */
    __SEV();
}

void i2c_request(void) {
    i2c_lock();

    if (!(users++)) {
        clocks_hw->wake_en0 |= CLOCKS_WAKE_EN0_CLK_SYS_I2C0_BITS;
        clocks_hw->sleep_en0 |= CLOCKS_SLEEP_EN0_CLK_SYS_I2C0_BITS;

        i2c_init(i2c0, 400000);
        gpio_set_function(16, GPIO_FUNC_I2C);
        gpio_set_function(17, GPIO_FUNC_I2C);
        gpio_pull_up(16);
        gpio_pull_up(17);
    }
}

void i2c_release(void) {
    if (!(--users)) {
        i2c_deinit(i2c0);
        clocks_hw->wake_en0 &= ~CLOCKS_WAKE_EN0_CLK_SYS_I2C0_BITS;
        clocks_hw->sleep_en0 &= ~CLOCKS_SLEEP_EN0_CLK_SYS_I2C0_BITS;

    }
    i2c_unlock();
}
