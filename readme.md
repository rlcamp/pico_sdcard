## Instructions

These assume a vanilla ARM64 Mac running Sonoma, with Xcode or at least its command line tools, *without* Homebrew.

### Obtain prerequisites

- Build and install [https://cmake.org/download/] from source using `./configure && make && sudo make install`

- Download the latest Arm GNU Toolchain .pkg file (or 13.1 if on an Intel Mac) from [https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads] and install it

- in `~/Downloads/`, run `git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git && cd pico-sdk && git submodule update --init`

### Build this code

- `mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 && cd ..`
- `make -C build -j4`

### Upload and run this code

- Hold down BOOTSEL while plugging into USB
- `cp build/*.uf2 /Volumes/RP2350`
