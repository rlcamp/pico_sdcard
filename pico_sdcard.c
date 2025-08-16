#include "pico/stdio_uart.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "RP2350.h"

#include "rp2350_sdcard.h"

#include <stdio.h>

void yield(void) {
    __DSB(); __WFE();
}

static void print_block(const unsigned char buf[]) {
    for (size_t ia = 0; ia < 32; ia++) {
        for (size_t ib = 0; ib < 16; ib++)
            dprintf(2, " %04X", buf[ib + 16 * ia]);
        dprintf(2, "\r\n");
    }
}

int main(void) {
//    set_sys_clock_48mhz();
    set_sys_clock_hz(96000000, true);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    /* enable sevonpend so that we don't need a nearly empty isr */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    stdio_uart_init();
    dprintf(2, "hello world stderr\r\n");

    do {
        if (-1 == spi_sd_init()) break;
        static unsigned char buf[512];
        if (-1 == spi_sd_read_blocks(buf, 1, 0)) break;
        print_block(buf);

        if (-1 == spi_sd_read_blocks(buf, 1, 1)) break;
        print_block(buf);

        if (-1 == spi_sd_write_blocks(buf, 1, 1)) break;

        if (-1 == spi_sd_read_blocks(buf, 1, 1)) break;
        print_block(buf);
    } while(0);

    gpio_put(22, 0);

    while (1) yield();
    NVIC_SystemReset();
}
