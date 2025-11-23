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

execute_process(COMMAND git describe --always --dirty --exclude '*'
                OUTPUT_VARIABLE GIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(COMMAND sh -c "TZ=UTC git log -1 --format=%cd --date=format-local:'%Y%m%dT%H%MZ'"
                OUTPUT_VARIABLE GIT_STAMP
                OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(COMMAND sh -c "TZ=UTC git log -1 --format=%cd --date=format-local:'%sULL'"
                OUTPUT_VARIABLE GIT_TIME
                OUTPUT_STRIP_TRAILING_WHITESPACE)

# initialize the SDK directly
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(pico_sdcard
    main.c
    rp2350_sdcard.c
    diskio.c
    ff.c
    ffunicode.c
    cooperative_fatfs.c
    rp2350_cooperative_uart.c
    dprintf.c
)

# add some warning flags TODO: figure out how to make these only apply to the above files
target_compile_options(pico_sdcard PRIVATE -Wall -Wextra -Wshadow -Wdouble-promotion)

# make git hash and time available to c preprocessor
target_compile_options(pico_sdcard PRIVATE -DGIT_VERSION="${GIT_HASH} \(${GIT_STAMP}\)" -DGIT_TIME=${GIT_TIME})

pico_generate_pio_header(pico_sdcard ${CMAKE_CURRENT_LIST_DIR}/rp2350_sdcard.pio)

# pull in common dependencies
target_link_libraries(pico_sdcard pico_stdlib cmsis_core hardware_dma hardware_pio hardware_spi hardware_i2c)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_sdcard)
