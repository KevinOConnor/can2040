cmake_minimum_required(VERSION 3.13..3.27)

if(NOT COMMAND pico_sdk_init)
	# pico_sdk_import.cmake is a single file copied from this SDK note: this
	# must happen before project()
	include(pico_sdk_import.cmake)
endif(NOT COMMAND pico_sdk_init)

project(
	can2040
	LANGUAGES C CXX # Note: picotool requires C++
	VERSION 1.6.0
)

if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(CAN2040_IS_IMPORTED 1)
endif(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)

if(CAN2040_IS_IMPORTED)
	if(NOT PICO_SDK_PATH)
		message(
			FATAL_ERROR "This module must be included after pico_sdk_init()"
		)
	endif(NOT PICO_SDK_PATH)
else(CAN2040_IS_IMPORTED)
	set(FAMILY
		rp2040
		CACHE STRING "family to use"
	)
	set(BOARD
		pico_sdk
		CACHE STRING "board to use"
	)

	pico_sdk_init()

	set(CMAKE_C_STANDARD 11)
	set(CMAKE_C_STANDARD_REQUIRED 1)

	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED 1)

endif(CAN2040_IS_IMPORTED)

add_library(can2040 INTERFACE)
add_library(can2040::can2040 ALIAS can2040)

target_sources(can2040 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../src/can2040.c)

target_link_libraries(
	can2040 INTERFACE pico_stdlib hardware_pio cmsis_core hardware_irq
)

target_include_directories(can2040 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

if(NOT CAN2040_IS_IMPORTED)
	add_subdirectory(../examples ${CMAKE_CURRENT_BINARY_DIR}/_examples)
endif(NOT CAN2040_IS_IMPORTED)
