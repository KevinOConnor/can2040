// Low-level definitions for CAN2040 functions
//
// Copyright (C) 2022-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Dmitrii Sharshakov <d3dx12.xx@gmail.com>
// SPDX-License-Identifier: GPL-3.0-only

#ifndef _CAN2040_LL_H
#define _CAN2040_LL_H

#include "can2040.h"
#include <stdint.h> // uint32_t

void can2040_ll_data_state_clear_bits(struct can2040 *cd);
void can2040_ll_data_state_go_discard(struct can2040 *cd);
void can2040_ll_pio_set_clkdiv(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate);
void can2040_ll_pio_sm_setup(struct can2040 *cd);

#endif // _CAN2040_LL_H
