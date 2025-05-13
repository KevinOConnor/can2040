# Demo cmake rules for the can2040 example using Pico SDK
#
# This is a minimal set of build rules.  See the PICO SDK
# documentation for more information.  In brief, from a Unix
# command-line one would issue commands similar to:
#   mkdir build
#   cd build
#   cmake -D PICO_BOARD=pico ..
#   make
# Then flash the resulting canexample.uf2 to the target board.

cmake_minimum_required(VERSION 3.13)

# Download SDK from git (or "cmake -D PICO_SDK_PATH=/some/path ..." to avoid)
set(PICO_SDK_FETCH_FROM_GIT on)

include(pico_sdk_import.cmake)

project(can2040_example)

pico_sdk_init()

add_executable(canexample
    canexample.c
    ../src/can2040.c
)

pico_set_binary_type(canexample copy_to_ram)
target_link_libraries(canexample pico_stdlib)
pico_enable_stdio_usb(canexample 1)
pico_enable_stdio_uart(canexample 0)

pico_add_extra_outputs(canexample)
