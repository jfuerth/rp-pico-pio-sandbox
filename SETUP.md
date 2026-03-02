# Setting up for building this software

## First-time project setup

This was one-time setup that doesn't need to be repeated. It's just here for reference
in case I want to create another new project in the future.

I followed the instructions in [pico-sdk](https://github.com/raspberrypi/pico-sdk?tab=readme-ov-file):

```
sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Then I opted for the "automatic download from GitHub" option for geting the SDK:

CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

# The list of boards will be in build/_deps/pico_sdk-src/src/boards/include/boards/ once you run cmake and it downloads the SDK
set(PICO_BOARD waveshare_rp2040_zero)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()
```

And added a source file to the above:

CMakeLists.txt (continued):
```cmake
add_executable(pio_capacitive_scanner
    pio_capacitive_scanner.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico_capacitive_scanner pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_capacitive_scanner)
```

## Building the binary

Generate the build directory (takes a minute because it downloads and builds the pico-sdk):

```
mkdir build
cd build
cmake ..
```

Make the binaries (still in the build directory):

```
make
```

## Uploading to the Pi Pico

After you've successfully built the binary:

1. Get the micro into bootloader mode:
    * Hold down the BOOT button while plugging it into the computer's USB port (release after it's plugged in), OR
    * While holding down the BOOT button, tap RESET.
2. Upload the .uf2 file by copying to the Pico when it appears as a mass-storage device: 
```
cp pio_capacitive_scanner.uf2 /media/$(whoami)/RPI-RP2/
```