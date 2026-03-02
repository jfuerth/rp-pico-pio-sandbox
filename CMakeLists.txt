cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# The list of boards will be in build/_deps/pico_sdk-src/src/boards/include/boards/ once you run cmake and it downloads the SDK
set(PICO_BOARD waveshare_rp2040_zero)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(pio_capacitive_scanner
    pio_capacitive_scanner.c
)

# Generate PIO header from .pio file
pico_generate_pio_header(pio_capacitive_scanner ${CMAKE_CURRENT_LIST_DIR}/capacitive_touch.pio)

# Add pico_stdlib library which aggregates commonly used features
# Also add hardware_pio for PIO support
target_link_libraries(pio_capacitive_scanner 
    pico_stdlib
    hardware_pio
)

# enable usb output, disable uart output
pico_enable_stdio_usb(pio_capacitive_scanner 1)
pico_enable_stdio_uart(pio_capacitive_scanner 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pio_capacitive_scanner)
