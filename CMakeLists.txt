# invoke using: mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 && cd ..

cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED ENV{PICO_SDK_PATH})
  set(ENV{PICO_SDK_PATH} "~/Downloads/pico-sdk/")
endif()

if (NOT DEFINED PICO_TOOLCHAIN_PATH)
    execute_process(
        COMMAND bash -c "dirname $((find $\{HOME\}/Downloads -name arm-none-eabi-gcc | sort; find /Applications/ArmGNUToolchain -name arm-none-eabi-gcc | sort) | tail -n1)"
        OUTPUT_VARIABLE PICO_TOOLCHAIN_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

# initialize the SDK directly
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# add some warning flags
set("CMAKE_C_FLAGS" "${CMAKE_C_FLAGS}  -Wall -Wextra -Wshadow")

# rest of your project
add_executable(pico_sdcard
    pico_sdcard.c
    rp2350_sdcard.c
    diskio.c
    ff.c
    ffunicode.c
    cortex_m_cooperative_multitasking.c
    rp2350_cooperative_uart.c
    rp2350_cooperative_i2c.c
    dprintf.c
    rp2350_ds3231.c
    rp2350_tsys01.c
    rp2350_kellerld.c
)

# pull in common dependencies
target_link_libraries(pico_sdcard pico_stdlib cmsis_core hardware_dma hardware_pio hardware_spi hardware_i2c)

pico_enable_stdio_uart(pico_sdcard 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_sdcard)
