/* this is going to start out as 1-bit spi mode, roughly following the logic in the
 equivalent samd51 code, but will then switch to 4-bit sdio mode using pio, with a set of
 pins chosen to allow either of those modes to work with no hardware change */
#include "hardware/sync.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "RP2350.h"

#include "rp2350_sdcard.h"

static unsigned requested_baud_rate = 0;

#include <stdio.h>

size_t card_overhead_numerator = 0, card_overhead_denominator = 0;
unsigned long microseconds_in_wait = 0;

__attribute((weak)) volatile unsigned char verbose = 0;

__attribute((weak)) void yield(void) {
    __DSB(); __WFE();
}

static void cs_low(void) {
    gpio_put(15, 0);
}

static void cs_high(void) {
    gpio_put(15, 1);
}

static uint8_t spi_receive_one_byte_with_rx_enabled(void) {
    uint8_t ret = 0;
    spi_read_blocking(spi1, 0xFF, &ret, 1);
    card_overhead_numerator++;
    return ret;
}

static uint8_t r1_response(void) {
    uint8_t result, attempts = 0;
    /* sd and mmc agree on max 8 attempts, mmc requires at least two attempts */
    while (0xFF == (result = spi_receive_one_byte_with_rx_enabled()) && attempts++ < 8);

    return result;
}

static unsigned char crc7_left_shifted(const unsigned char * restrict const message, const size_t length) {
    const unsigned char polynomial = 0b10001001;
    unsigned char crc = 0;

    for (size_t ibyte = 0; ibyte < length; ibyte++) {
        crc ^= message[ibyte];

        for (size_t ibit = 0; ibit < 8; ibit++)
            crc = (crc & 0x80u) ? (crc << 1) ^ (polynomial << 1) : (crc << 1);
    }

    return crc & 0xfe;
}

static void send_command_with_crc7(const uint8_t cmd, const uint32_t arg) {
    unsigned char msg[6] = { cmd | 0x40, arg >> 24, arg >> 16, arg >> 8, arg, 0x01 };
    msg[5] |= crc7_left_shifted(msg, 5);

    spi_write_blocking(spi1, msg, 6);
    card_overhead_numerator += 6;
}

static uint8_t command_and_r1_response(const uint8_t cmd, const uint32_t arg) {
    send_command_with_crc7(cmd, arg);
    return r1_response();
}

static void wait_for_card_ready(void) {
    const unsigned long timerawl_prior = timer_hw->timerawl;
    spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    uint16_t ret;
    do {
        spi_read16_blocking(spi1, 0xFFFF, &ret, 1);
        card_overhead_numerator += 2;
    } while (ret != 0xFFFF);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    microseconds_in_wait += timer_hw->timerawl - timerawl_prior;
}

static uint32_t spi_receive_uint32be(void) {
    uint32_t ret;
    spi_read_blocking(spi1, 0xFF, (void *)&ret, 4);
    card_overhead_numerator += 4;
    return __builtin_bswap32(ret);
}

void spi_sd_restore_baud_rate(void) {
    requested_baud_rate = clock_get_hz(clk_peri) / 2U;
}

int spi_sd_init(unsigned baud_rate_reduction) {
    spi_init(spi1, 400000);
    gpio_set_function(10, GPIO_FUNC_SPI);
    gpio_set_function(11, GPIO_FUNC_SPI);
    gpio_set_function(12, GPIO_FUNC_SPI);

    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    cs_high();

    /* clear miso */
    cs_low();
    spi_write_blocking(spi1, &(uint8_t){ 0xFF }, 1);
    cs_high();

    /* and then clock out at least 74 cycles at 100-400 kBd with cs pin held high */
    spi_write_blocking(spi1, (unsigned char[10]) { [0 ... 9] = 0xFF }, 10);

    /* cmd0, software reset. TODO: is looping this even necessary? */
    for (size_t ipass = 0;; ipass++) {
        /* if card likely not present, give up */
        if (ipass > 1024) {
            dprintf(2, "%s(%d): fail\r\n", __func__, __LINE__);
            return -1;
        }
        cs_low();

        /* send cmd0 */
        const uint8_t cmd0_r1_response = command_and_r1_response(0, 0);
        cs_high();

        if (0x01 == cmd0_r1_response) break;

        /* give other stuff a chance to run if we are looping */
        __SEV(); yield();
    }

    if (verbose >= 2)
        dprintf(2, "%s: cmd0 success\r\n", __func__);

    /* cmd8, check voltage range and test pattern */
    for (size_t ipass = 0;; ipass++) {
        if (ipass > 3) {
            /* TODO: find out how many times we should try this before giving up */
            spi_deinit(spi1);
            return -1;
        }
        cs_low();
        wait_for_card_ready();

        const uint8_t r1_response = command_and_r1_response(8, 0x1AA);

        if (0x1 != r1_response) {
            cs_high();
            continue;
        }

        const uint32_t response = spi_receive_uint32be();
        cs_high();

        if (0x1AA == response) break;
    }

    if (verbose >= 2)
        dprintf(2, "%s: cmd8 success\r\n", __func__);

    /* cmd59, re-enable crc feature, which is disabled by cmd0 */
    cs_low();
    wait_for_card_ready();
    if (command_and_r1_response(59, 1) > 1) {
        cs_high();
        spi_deinit(spi1);
        return -1;
    }
    cs_high();

    if (verbose >= 2)
        dprintf(2, "%s: cmd59 success\r\n", __func__);

    /* cmd55, then acmd41, init. must loop this until the response is 0 */
    for (size_t ipass = 0;; ipass++) {
        if (ipass > 2500) {
            spi_deinit(spi1);
            dprintf(2, "%s: giving up\r\n", __func__);
            return -1;
        }
        cs_low();
        wait_for_card_ready();

        const uint8_t cmd55_r1_response = command_and_r1_response(55, 0);
        cs_high();

        if (cmd55_r1_response > 1) continue;

        cs_low();
        wait_for_card_ready();

        const uint8_t acmd41_r1_response = command_and_r1_response(41, 1U << 30);
        cs_high();

        if (!acmd41_r1_response) break;
    }

    if (verbose >= 2)
        dprintf(2, "%s: cmd55+acmd41 success\r\n", __func__);

    spi_deinit(spi1);
    requested_baud_rate = clock_get_hz(clk_peri) / (2U + baud_rate_reduction);
    const unsigned actual_baud = spi_init(spi1, requested_baud_rate);
    if (verbose >= 1)
        dprintf(2, "%s: baud rate set to %u\r\n", __func__, actual_baud);

    /* TODO: if any of the following fail, restart the procedure with a lower baud rate */
    do {
        /* cmd58, read ocr register */
        cs_low();
        wait_for_card_ready();
        if (command_and_r1_response(58, 0) > 1) break;

        const unsigned int ocr = spi_receive_uint32be();
        (void)ocr;
        cs_high();

        /* cmd16, set block length to 512 */
        cs_low();
        wait_for_card_ready();
        if (command_and_r1_response(16, 512) > 1) break;
        cs_high();

        /* we get here on overall success of this function */
        cs_high();
        spi_deinit(spi1);

        if (verbose >= 1)
            dprintf(2, "%s: success\r\n", __func__);
        return 0;
    } while(0);

    /* we get here on failure */
    cs_high();
    spi_deinit(spi1);
    return -1;
}

int spi_sd_write_blocks_start(unsigned long long block_address) {
    spi_init(spi1, requested_baud_rate);
    cs_low();
    wait_for_card_ready();

    const uint8_t response = command_and_r1_response(25, block_address);
    if (response != 0) {
        cs_high();
        spi_deinit(spi1);
        return -1;
    }

    /* extra byte prior to data packet */
    spi_write_blocking(spi1, (unsigned char[1]) { 0xff }, 1);
    card_overhead_numerator++;
    return 0;
}

void spi_sd_write_blocks_end(void) {
    /* send stop tran token */
    spi_write_blocking(spi1, (unsigned char[2]) { 0xfd, 0xff }, 2);
    card_overhead_numerator += 2;

    wait_for_card_ready();

    cs_high();
    spi_deinit(spi1);
}

int spi_sd_write_pre_erase(unsigned long blocks) {
    spi_init(spi1, requested_baud_rate);
    cs_low();
    wait_for_card_ready();

    const uint8_t cmd55_r1_response = command_and_r1_response(55, 0);
    cs_high();

    if (cmd55_r1_response > 1) {
        spi_deinit(spi1);
        return -1;
    }

    cs_low();
    wait_for_card_ready();

    const uint8_t acmd23_r1_response = command_and_r1_response(23, blocks);

    cs_high();
    spi_deinit(spi1);

    return acmd23_r1_response ? -1 : 0;
}

int spi_sd_write_some_blocks(const void * buf, const unsigned long blocks) {
    for (size_t iblock = 0; iblock < blocks; iblock++) {
        const unsigned char * block = buf ? (void *)((unsigned char *)buf + 512 * iblock) : NULL;

        while (spi_is_busy(spi1));
        spi_write_blocking(spi1, (unsigned char[1]) { 0xfc }, 1);
        card_overhead_numerator++;

        while (spi_is_busy(spi1));

        spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

        const uint dma_tx = dma_claim_unused_channel(true);

        static const uint16_t zero_word = 0;
        dma_channel_config cfg = dma_channel_get_default_config(dma_tx);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        channel_config_set_dreq(&cfg, spi_get_dreq(spi1, true));
        channel_config_set_read_increment(&cfg, block ? true : false);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_bswap(&cfg, true);
        dma_channel_configure(dma_tx, &cfg, &spi_get_hw(spi1)->dr, block ? block : (void *)&zero_word, 256, false);

        dma_channel_acknowledge_irq1(dma_tx);

        /* since we are using sevonpend, enable the irq source but disable in nvic */
        dma_channel_set_irq1_enabled(dma_tx, true);
        irq_set_enabled(DMA_IRQ_1, false);

        /* compute a CCITT16 CRC on the bytes flowing through the rx dma */
        dma_sniffer_enable(dma_tx, 0x2, true);
        dma_sniffer_set_byte_swap_enabled(true);
        dma_sniffer_set_data_accumulator(0);

        /* start the dma channel */
        dma_start_channel_mask(1u << dma_tx);

        /* do other things and then sleep, while waiting for dma to finish */
        while (dma_channel_is_busy(dma_tx))
            yield();

        card_overhead_numerator += 512;
        card_overhead_denominator += 512;

        /* disable and clear the irq that caused wfe to return due to sevonpend */
        dma_channel_acknowledge_irq1(dma_tx);
        dma_channel_set_irq1_enabled(dma_tx, false);
        irq_clear(DMA_IRQ_1);

        /* retrieve the crc that we calculated on the bytes as they came in */
        const uint16_t crc_dma = dma_sniffer_get_data_accumulator();

        dma_channel_unclaim(dma_tx);
        dma_sniffer_disable();

        /* wait for the rest of the spi tx fifo to drain, with wfe inhibited because it
         will not be accompanied by an interrupt that would wake the processor */
        while (spi_is_busy(spi1)) {
            __sev();
            yield();
        }

        /* write the calculated crc out to the card so it can validate it */
        spi_write16_blocking(spi1, &crc_dma, 1);

        card_overhead_numerator += 2;

        /* change format back to 8 bit */
        while (spi_is_busy(spi1));
        spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

        /* read one byte response from card to see if it validated the crc */
        uint8_t response;
        spi_read_blocking(spi1, 0xff, &response, 1);
        card_overhead_numerator++;
        response &= 0b11111;

        const unsigned timerawl_prior = timer_hw->timerawl;
        wait_for_card_ready();
        const unsigned timerawl_elapsed = timer_hw->timerawl - timerawl_prior;

        if (verbose >= 2)
            dprintf(2, "%s: %u us\r\n", __func__, timerawl_elapsed);

        if (0b00101 != response) {
            if (0b01011 == response)
                dprintf(2, "%s: bad crc (sent 0x%04X)\r\n", __func__, crc_dma);
            else
                dprintf(2, "%s: error 0x%x\r\n", __func__, response);

            cs_high();
            spi_deinit(spi1);
            return -1;
        }
    }

    return 0;
}

int spi_sd_write_blocks(const void * buf, const unsigned long blocks, const unsigned long long block_address) {
    if (-1 == spi_sd_write_blocks_start(block_address) ||
        -1 == spi_sd_write_some_blocks(buf, blocks))
        return -1;

    spi_sd_write_blocks_end();

    return 0;
}

int spi_sd_read_blocks(void * buf, unsigned long blocks, unsigned long long block_address) {
    spi_init(spi1, requested_baud_rate);
    cs_low();
    wait_for_card_ready();

    /* send cmd17 or cmd18 */
    if (command_and_r1_response(blocks > 1 ? 18 : 17, block_address) != 0) {
        cs_high();
        spi_deinit(spi1);
        dprintf(2, "%s(%d): fail\r\n", __func__, __LINE__);
        return -1;
    }

    /* clock out the response in 1 + 512 + 2 byte blocks */
    for (size_t iblock = 0; iblock < blocks; iblock++) {
        uint8_t result;
        /* this can loop for a while */
        while (0xFF == (result = spi_receive_one_byte_with_rx_enabled()));

        /* when we break out of the above loop, we've read the Data Token byte */
        if (0xFE != result) {
            cs_high();
            spi_deinit(spi1);
            dprintf(2, "%s(%d): fail\r\n", __func__, __LINE__);
            return -1;
        }

        spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

        uint16_t * restrict const block = ((uint16_t *)buf) + 256 * iblock;

        const uint dma_rx = dma_claim_unused_channel(true);
        const uint dma_tx = dma_claim_unused_channel(true);

        /* there has got to be a better way to do this than clock out bytes */
        dma_channel_config cfg = dma_channel_get_default_config(dma_tx);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        channel_config_set_dreq(&cfg, spi_get_dreq(spi1, true));
        channel_config_set_read_increment(&cfg, false);
        dma_channel_configure(dma_tx, &cfg, &spi_get_hw(spi1)->dr, &(uint16_t) { 0xFFFF }, 256, false);

        cfg = dma_channel_get_default_config(dma_rx);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
        channel_config_set_dreq(&cfg, spi_get_dreq(spi1, false));
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_bswap(&cfg, true);
        dma_channel_configure(dma_rx, &cfg, block, &spi_get_hw(spi1)->dr, 256, false);

        dma_channel_acknowledge_irq1(dma_rx);

        /* since we are using sevonpend, enable the irq source but disable in nvic */
        dma_channel_set_irq1_enabled(dma_rx, true);
        irq_set_enabled(DMA_IRQ_1, false);

        /* compute a CCITT16 CRC on the bytes flowing through the rx dma */
        dma_sniffer_enable(dma_rx, 0x2, true);
        dma_sniffer_set_byte_swap_enabled(false);
        dma_sniffer_set_data_accumulator(0);

        /* start both dma channels simultaneously */
        dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));

        unsigned wakes = 0;

        /* do other things and then sleep, while waiting for dma to finish */
        while (dma_channel_is_busy(dma_rx)) {
            yield();
            wakes++;
        }

        /* disable and clear the irq that caused wfe to return due to sevonpend */
        dma_channel_acknowledge_irq1(dma_rx);
        dma_channel_set_irq1_enabled(dma_rx, false);
        irq_clear(DMA_IRQ_1);

        /* retrieve the crc that we calculated on the bytes as they came in */
        const uint16_t crc_dma = dma_sniffer_get_data_accumulator();

        dma_channel_unclaim(dma_rx);
        dma_channel_unclaim(dma_tx);
        dma_sniffer_disable();

        /* read the crc reported by the sd card */
        uint16_t crc_received;
        spi_read16_blocking(spi1, 0xFFFF, (void *)&crc_received, 1);

        spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

        if (verbose >= 2)
            dprintf(2, "%s: received crc 0x%04X, dma 0x%04X, %u wakes\r\n",
                    __func__, crc_received, crc_dma, wakes);

        card_overhead_numerator += 512 + 2 + 1;

        if (crc_received != crc_dma) {
            cs_high();
            spi_deinit(spi1);
            dprintf(2, "%s: bad crc\r\n", __func__);
            return -1;
        }
    }

    /* if we sent cmd18, send cmd12 to stop */
    if (blocks > 1) {
        send_command_with_crc7(12, 0);

        /* CMD12 wants an extra byte prior to the response */
        spi_write_blocking(spi1, (unsigned char[1]) { 0xff }, 1);
        card_overhead_numerator += 2;

        (void)r1_response();
        wait_for_card_ready();
    }

    cs_high();
    spi_deinit(spi1);

    return 0;
}
