extern unsigned long long unix_microseconds_at_ref;
extern unsigned long long uptime_microseconds_at_ref;

int ds3231_to_sys(void);

int gpzda_to_sys(const char * line, const unsigned baud_rate, const unsigned long long uptime_microseconds_at_end_of_line);

int sys_to_ds3231(void);
