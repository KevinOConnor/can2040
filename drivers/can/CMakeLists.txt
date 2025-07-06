# Copyright (c) 2025 Dmitrii Sharshakov <d3dx12.xx@gmail.com>
# SPDX-License-Identifier: GPL-3.0-only OR Apache-2.0

if(CONFIG_CAN_CAN2040)
zephyr_include_directories(../../src)
zephyr_sources(../../src/can2040.c)

zephyr_sources(can_can2040.c)
endif()
