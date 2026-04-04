cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_FETCH_FROM_GIT on)

# Use a local SDK path instead of fetching from git
# Option 1: Set path directly (update this path to your local SDK)
# set(PICO_SDK_PATH "/path/to/your/pico-sdk")

# Option 2: Use the already-fetched SDK in build directory
set(PICO_SDK_PATH "$ENV{HOME}/prg/3rdparty/pico-sdk")

# The list of boards will be in build/_deps/pico_sdk-src/src/boards/include/boards/ once you run cmake and it downloads the SDK
#set(PICO_BOARD waveshare_rp2040_zero)
set(PICO_BOARD waveshare_rp2350_tiny) # not exactly Waveshare Core2350B but close enough?

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

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
    hardware_dma
)

# enable usb output, disable uart output
pico_enable_stdio_usb(pio_capacitive_scanner 1)
pico_enable_stdio_uart(pio_capacitive_scanner 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pio_capacitive_scanner)

# Test executable for DMA chaining with simpler setup
add_executable(pio_dma_chain_test
    pio_dma_chain_test.c
)

# Generate PIO header from .pio file
pico_generate_pio_header(pio_dma_chain_test ${CMAKE_CURRENT_LIST_DIR}/pio_count.pio)

target_link_libraries(pio_dma_chain_test 
    pico_stdlib
    hardware_pio
    hardware_dma
)

pico_enable_stdio_usb(pio_dma_chain_test 1)
pico_enable_stdio_uart(pio_dma_chain_test 0)
pico_add_extra_outputs(pio_dma_chain_test)

# Simple test to verify DMA chaining reload behavior
add_executable(dma_chain_reload_test
    dma_chain_reload_test.c
)

target_link_libraries(dma_chain_reload_test 
    pico_stdlib
    hardware_dma
)

pico_enable_stdio_usb(dma_chain_reload_test 1)
pico_enable_stdio_uart(dma_chain_reload_test 0)
pico_add_extra_outputs(dma_chain_reload_test)
