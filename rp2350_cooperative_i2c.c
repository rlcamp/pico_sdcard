#include "rp2350_cooperative_i2c.h"
#include "RP2350.h"

/* intentionally cooperative only mutex-like thing, can be made actually thread safe but no need */
static volatile unsigned char lock = 0;

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
    /* TODO: postpone i2c init until here if nobody was using it */
    i2c_lock();
}

void i2c_release(void) {
    /* TODO: deinit i2c if nobody else still needs it */
    i2c_unlock();
}

