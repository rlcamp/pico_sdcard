#include "rp2350_sdcard.h"

int card_request(void);
void card_release(void);

void card_lock(void);
void card_unlock(void);

int ls(const char * path);
int cat(const char * path);

/* third party includes */
#include "ff.h"

extern FATFS * fs;
