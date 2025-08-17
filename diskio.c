/* minimum viable glue code between fatfs and spi sd code */

/* definitions of datatypes and of functions expected by ff.c */
#include "ff.h"
#include "diskio.h"

/* block device implementation code being wrapped by this */
#include "rp2350_sdcard.h"

/* needed for INT_MAX, this will go away */
#include <limits.h>

#include <stdio.h>

size_t fatfs_sectors_read = 0, fatfs_sectors_written = 0;

__attribute((weak)) unsigned char verbose = 0;

unsigned char diskio_initted = 0;

DSTATUS disk_status(BYTE pdrv) {
    (void)pdrv;
    return diskio_initted ? 0 : STA_NOINIT;
}

static LBA_t block_cache_sectors[64];
static unsigned char block_cache[64][512];
static const size_t B = sizeof(block_cache) / sizeof(block_cache[0]);
static size_t icache = 0;

DSTATUS disk_initialize(BYTE pdrv) {
    (void)pdrv;
    if (!diskio_initted) {
        if (-1 == spi_sd_init()) return STA_NOINIT;
    }

    __builtin_memset(block_cache_sectors, 0, sizeof(block_cache_sectors));

    diskio_initted = 1;
    return 0;
}

static LBA_t deferred_zeros_sector_start = 0;
static UINT deferred_zeros_sector_count = 0;

static DRESULT flush_deferred_zeros(void) {
    const UINT count = deferred_zeros_sector_count;
    deferred_zeros_sector_count = 0;

    fatfs_sectors_written += count;

    if (verbose >= 1)
        dprintf(2, "%s: writing %u blocks of deferred zeros\r\n", __func__, count);
    if (-1 == spi_sd_write_blocks(NULL, count, deferred_zeros_sector_start))
        return RES_ERROR;
    return 0;
}

static void cache_block(const BYTE * buff, LBA_t sector) {
    for (size_t icache_search = 0; icache_search < B; icache_search++)
        if (sector && block_cache_sectors[icache_search] == sector) {
            icache = icache_search;
            break;
        }

    __builtin_memcpy(block_cache[icache], buff, 512);
    block_cache_sectors[icache] = sector;
    icache = (icache + 1) % B;
}
void print_block(const unsigned char buf[]);

DRESULT disk_read(BYTE pdrv, BYTE * buff, LBA_t sector, UINT count) {
    (void)pdrv;

    if (deferred_zeros_sector_count) {
        const DRESULT res = flush_deferred_zeros();
        if (res) return res;
    }

    for (size_t icache_search = 0; icache_search < B; icache_search++)
        if (1 == count && sector && block_cache_sectors[icache_search] == sector) {
            if (verbose >= 2)
                dprintf(2, "%s(%d): reusing cached block %u at %u\r\n", __func__, __LINE__, (unsigned)sector, icache_search);
            __builtin_memcpy(buff, block_cache[icache_search], 512);
            return 0;
        }

    if (verbose >= 1)
        dprintf(2, "%s(%d): reading %u blocks starting at %u\r\n", __func__, __LINE__, count, (unsigned)sector);

    fatfs_sectors_read += count;

    /* this will block, but will internally call yield() and __WFI() */
    if (-1 == spi_sd_read_blocks(buff, count, sector)) return RES_ERROR;

    cache_block(buff, sector);
    return 0;
}

static int buffer_points_to_all_zeros(const BYTE * buff, UINT count) {
    for (size_t ibyte = 0; ibyte < 512 * count; ibyte++)
        if (buff[ibyte]) return 0;
    return 1;
}

DRESULT disk_write(BYTE pdrv, const BYTE * buff, LBA_t sector, UINT count) {
    (void)pdrv;

    if (!deferred_zeros_sector_count || sector == deferred_zeros_sector_start + deferred_zeros_sector_count) {
        if (buffer_points_to_all_zeros(buff, count)) {
            if (!deferred_zeros_sector_count)
                deferred_zeros_sector_start = sector;
            deferred_zeros_sector_count += count;
            return 0;
        }
    }
    else if (deferred_zeros_sector_count) {
        const DRESULT res = flush_deferred_zeros();
        if (res) return res;
    }

    if (verbose >= 1)
        dprintf(2, "%s(%d): writing block(s) starting at %u\r\n", __func__, __LINE__, (unsigned)sector);

    fatfs_sectors_written += count;

    if (-1 == spi_sd_write_blocks(buff, count, sector)) return RES_ERROR;

    cache_block(buff, sector);
    return 0;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void * buff) {
    (void)pdrv;
    if (CTRL_SYNC == cmd) {
        if (deferred_zeros_sector_count) {
            const DRESULT res = flush_deferred_zeros();
            if (res) return res;
        }
        return 0;
    }
    else if (GET_BLOCK_SIZE == cmd)
        *(LBA_t *)buff = 1; /* TODO: populate this from actual */
    else if (GET_SECTOR_COUNT == cmd)
        *(LBA_t *)buff = INT_MAX; /* TODO: populate this from actual */
    else return RES_PARERR;
    return 0;
}
