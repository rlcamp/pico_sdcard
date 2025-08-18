/* call this before you start an i2c transaction. it returns with the lock held */
extern void i2c_request(void);

/* call this after you are done. it also releases the lock */
extern void i2c_release(void);

/* call these if you need to lock or unlock the mutex explicitly in between the above */
extern void i2c_lock(void);
extern void i2c_unlock(void);

/* this will return 0 immediately if it acquires the lock, or -1 immediately if it does not */
extern int i2c_lock_or_fail(void);
