// Software CANbus implementation for rp2040
//
// Copyright (C) 2022,2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include <string.h> // memset, memcpy
#include "RP2040.h" // hw_set_bits
#include "can2040.h" // can2040_setup, can2040_rx_cb, CAN2040_NOTIFY_ERROR, CAN2040_NOTIFY_RX, CAN2040_NOTIFY_TX, CAN2040_ID_EFF, CAN2040_ID_RTR
#include "hardware/regs/dreq.h" // DREQ_PIO0_RX1
#include "hardware/structs/dma.h" // dma_hw
#include "hardware/structs/iobank0.h" // iobank0_hw
#include "hardware/structs/padsbank0.h" // padsbank0_hw
#include "hardware/structs/pio.h" // pio0_hw, pio1_hw
#include "hardware/structs/resets.h" // RESETS_RESET_PIO0_BITS, RESETS_RESET_PIO1_BITS

/****************************************************************
 * rp2040 and low-level helper functions
 ****************************************************************/

// Compiler barrier to prevent reordering
#define barrier() __asm__ __volatile__("": : :"memory")

// Branch prediction hints
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

// Macro to calculate the number of elements in an array
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Macro to perform integer division with rounding up
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

// Helper functions for writing to and reading from memory-mapped I/O
static inline void writel(void *addr, uint32_t val) {
    barrier();
    *(volatile uint32_t *)addr = val;
}

static inline uint32_t readl(const void *addr) {
    uint32_t val = *(volatile const uint32_t *)addr;
    barrier();
    return val;
}

// rp2040 helper function to clear a hardware reset bit and wait until it's done
static void rp2040_clear_reset(uint32_t reset_bit) {
    if (resets_hw->reset & reset_bit) {
        hw_clear_bits(&resets_hw->reset, reset_bit);
        while (!(resets_hw->reset_done & reset_bit))
            ;
    }
}

// Helper to set the mode and pull-up/down configuration of a GPIO pin
static void rp2040_gpio_peripheral(uint32_t gpio, int func, int pull_up) {
    padsbank0_hw->io[gpio] = (
        PADS_BANK0_GPIO0_IE_BITS
        | (PADS_BANK0_GPIO0_DRIVE_VALUE_4MA << PADS_BANK0_GPIO0_DRIVE_MSB)
        | (pull_up > 0 ? PADS_BANK0_GPIO0_PUE_BITS : 0)
        | (pull_up < 0 ? PADS_BANK0_GPIO0_PDE_BITS : 0));
    iobank0_hw->io[gpio].ctrl = func << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

/****************************************************************
 * rp2040 PIO support
 ****************************************************************/

// Constants defining PIO clock and wake bits
#define PIO_CLOCK_PER_BIT 32
#define PIO_RX_WAKE_BITS 10

// Offsets into the PIO instruction memory for various synchronization points
#define can2040_offset_sync_found_end_of_message 2u
#define can2040_offset_sync_signal_start 4u
#define can2040_offset_sync_entry 6u
#define can2040_offset_sync_end 13u
#define can2040_offset_shared_rx_read 13u
#define can2040_offset_shared_rx_end 15u
#define can2040_offset_match_load_next 18u
#define can2040_offset_tx_conflict 24u
#define can2040_offset_match_end 25u
#define can2040_offset_tx_got_recessive 25u
#define can2040_offset_tx_write_pin 27u

// PIO program instructions for CANbus handling
static const uint16_t can2040_program_instructions[] = {
    0x0085, //  0: jmp    y--, 5
    0x0048, //  1: jmp    x--, 8
    0xe029, //  2: set    x, 9
    0x00cc, //  3: jmp    pin, 12
    0xc000, //  4: irq    nowait 0
    0x00c0, //  5: jmp    pin, 0
    0xc040, //  6: irq    clear 0
    0xe429, //  7: set    x, 9                   [4]
    0xf043, //  8: set    y, 3                   [16]
    0xc104, //  9: irq    nowait 4               [1]
    0x03c5, // 10: jmp    pin, 5                 [3]
    0x0307, // 11: jmp    7                      [3]
    0x0043, // 12: jmp    x--, 3
    0x20c4, // 13: wait   1 irq, 4
    0x4001, // 14: in     pins, 1
    0xa046, // 15: mov    y, isr
    0x01b2, // 16: jmp    x != y, 18             [1]
    0xc002, // 17: irq    nowait 2
    0x40eb, // 18: in     osr, 11
    0x4054, // 19: in     y, 20
    0xa047, // 20: mov    y, osr
    0x8080, // 21: pull   noblock
    0xa027, // 22: mov    x, osr
    0x0098, // 23: jmp    y--, 24
    0xa0e2, // 24: mov    osr, y
    0x6021, // 25: out    x, 1
    0x00df, // 26: jmp    pin, 31
    0xb801, // 27: mov    pins, x                [24]
    0x02d9, // 28: jmp    pin, 25                [2]
    0x0058, // 29: jmp    x--, 24
    0x6021, // 30: out    x, 1
    0x011b, // 31: jmp    27                     [1]
};

// Local names for PIO state machine IRQs for better readability
#define SI_MAYTX     PIO_IRQ0_INTE_SM0_BITS
#define SI_MATCHED   PIO_IRQ0_INTE_SM2_BITS
#define SI_ACKDONE   PIO_IRQ0_INTE_SM3_BITS
#define SI_RX_DATA   PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS
#define SI_TXPENDING PIO_IRQ0_INTE_SM1_BITS // Misc bit manually forced

/**
 * @brief Setup PIO "sync" state machine (state machine 0)
 *
 * This state machine handles synchronization of CANbus frames.
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_sync_setup(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[0];
    sm->execctrl = (
        cd->gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB
        | (can2040_offset_sync_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_sync_signal_start << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = (
        1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
        | cd->gpio_rx << PIO_SM0_PINCTRL_SET_BASE_LSB);
    sm->instr = 0xe080; // set pindirs, 0
    sm->pinctrl = 0;
    pio_hw->txf[0] = 9 + 6 * PIO_CLOCK_PER_BIT / 2;
    sm->instr = 0x80a0; // pull block
    sm->instr = can2040_offset_sync_entry; // jmp sync_entry
}

/**
 * @brief Setup PIO "rx" state machine (state machine 1)
 *
 * This state machine handles receiving CANbus frames.
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_rx_setup(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[1];
    sm->execctrl = (
        (can2040_offset_shared_rx_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = cd->gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0; // Flush FIFO on a restart
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
                     | PIO_RX_WAKE_BITS << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB
                     | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    sm->instr = can2040_offset_shared_rx_read; // jmp shared_rx_read
}

/**
 * @brief Setup PIO "match" state machine (state machine 2)
 *
 * This state machine handles matching received frames for acknowledgment.
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_match_setup(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[2];
    sm->execctrl = (
        (can2040_offset_match_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = cd->gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0;
    sm->instr = 0xe040; // set y, 0
    sm->instr = 0xa0e2; // mov osr, y
    sm->instr = 0xa02a; // mov x, !y  // Korrigiert: Komma zu Semikolon geändert
    sm->instr = can2040_offset_match_load_next; // jmp match_load_next
}

/**
 * @brief Setup PIO "tx" state machine (state machine 3)
 *
 * Diese Zustandsmaschine behandelt das Senden von CANbus-Frames.
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_tx_setup(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->execctrl = (
        cd->gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB
        | can2040_offset_tx_conflict << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_tx_conflict << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    sm->pinctrl = (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
                   | 1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB
                   | cd->gpio_tx << PIO_SM0_PINCTRL_SET_BASE_LSB
                   | cd->gpio_tx << PIO_SM0_PINCTRL_OUT_BASE_LSB);
    sm->instr = 0xe001; // set pins, 1
    sm->instr = 0xe081; // set pindirs, 1
}

/**
 * @brief Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 11 idle bits
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_sync_normal_start_signal(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    pio_hw->instr_mem[eom_idx] = 0xe12a; // set x, 10 [1]
}

/**
 * @brief Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 17 idle bits
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_sync_slow_start_signal(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    pio_hw->instr_mem[eom_idx] = 0xa127; // mov x, osr [1]
}

/**
 * @brief Test if PIO "rx" state machine has overflowed its FIFOs
 *
 * @param cd Pointer to the can2040 structure
 * @return int Non-zero if overflow has occurred, else zero
 */
static int pio_rx_check_stall(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    return pio_hw->fdebug & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1));
}

/**
 * @brief Set PIO "match" state machine to raise a "matched" signal on a bit sequence
 *
 * @param cd Pointer to the can2040 structure
 * @param match_key Bit sequence key to match
 */
static void pio_match_check(struct can2040 *cd, uint32_t match_key) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->txf[2] = match_key;
}

/**
 * @brief Calculate pos+bits identifier for PIO "match" state machine
 *
 * @param raw_bits Raw bit sequence
 * @param rx_bit_pos Bit position
 * @return uint32_t Calculated match key
 */
static uint32_t pio_match_calc_key(uint32_t raw_bits, uint32_t rx_bit_pos) {
    return (raw_bits & 0x1FFFFF) | ((-rx_bit_pos) << 21);
}

/**
 * @brief Cancel any pending checks on PIO "match" state machine
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_match_clear(struct can2040 *cd) {
    pio_match_check(cd, 0);
}

/**
 * @brief Flush and halt PIO "tx" state machine
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_tx_reset(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
    pio_hw->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
    pio_hw->irq = (SI_MATCHED | SI_ACKDONE) >> 8; // Clear PIO IRQ flags
    // Clear TX FIFO
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->shiftctrl = 0;
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
}

/**
 * @brief Queue a message for transmission on PIO "tx" state machine
 *
 * @param cd Pointer to the can2040 structure
 * @param data Pointer to data words to send
 * @param count Number of data words
 */
static void pio_tx_send(struct can2040 *cd, uint32_t *data, uint32_t count) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_tx_reset(cd);
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0x6021; // out x, 1
    uint32_t i;
    for (i = 0; i < count; i++)
        pio_hw->txf[3] = data[i];
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = 0x6021; // out x, 1
    sm->instr = can2040_offset_tx_write_pin; // jmp tx_write_pin
    sm->instr = 0x20c0; // wait 1 irq, 0
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;
}

/**
 * @brief Set PIO "tx" state machine to inject an ack after a CRC match
 *
 * @param cd Pointer to the can2040 structure
 * @param match_key Bit sequence key for matching
 */
static void pio_tx_inject_ack(struct can2040 *cd, uint32_t match_key) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_tx_reset(cd);
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0xc023; // irq wait 3
    pio_hw->txf[3] = 0x7FFFFFFF;
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = 0x6021; // out x, 1
    sm->instr = can2040_offset_tx_write_pin; // jmp tx_write_pin
    sm->instr = 0x20c2; // wait 1 irq, 2
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;

    pio_match_check(cd, match_key);
}

/**
 * @brief Check if the PIO "tx" state machine unexpectedly finished a transmit attempt
 *
 * @param cd Pointer to the can2040 structure
 * @return int Non-zero if failure detected, else zero
 */
static int pio_tx_did_fail(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    // Check for passive/dominant bit conflict without parser noticing
    if (pio_hw->sm[3].addr == can2040_offset_tx_conflict)
        return !(pio_hw->intr & SI_RX_DATA);
    // Check for unexpected drain of transmit queue without parser noticing
    return (!(pio_hw->flevel & PIO_FLEVEL_TX3_BITS)
            && (pio_hw->intr & (SI_MAYTX | SI_RX_DATA)) == SI_MAYTX);
}

/**
 * @brief Enable host IRQs for state machine signals
 *
 * @param cd Pointer to the can2040 structure
 * @param sm_irqs IRQ bits to enable
 */
static void pio_irq_set(struct can2040 *cd, uint32_t sm_irqs) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->inte0 = sm_irqs | SI_RX_DATA;
}

/**
 * @brief Completely disable host IRQs
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_irq_disable(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->inte0 = 0;
}

/**
 * @brief Return current host IRQ mask
 *
 * @param cd Pointer to the can2040 structure
 * @return uint32_t Current IRQ mask
 */
static uint32_t pio_irq_get(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    return pio_hw->inte0;
}

/**
 * @brief Raise the txpending flag to signal pending transmission
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_signal_set_txpending(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->irq_force = SI_TXPENDING >> 8;
}

/**
 * @brief Clear the txpending flag
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_signal_clear_txpending(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->irq = SI_TXPENDING >> 8;
}

/**
 * @brief Setup all PIO state machines
 *
 * @param cd Pointer to the can2040 structure
 */
static void pio_sm_setup(struct can2040 *cd) {
    // Reset state machines and clear IRQs
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->ctrl = PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS;
    pio_hw->fdebug = 0xFFFFFFFF;
    pio_hw->irq = 0xFF;
    pio_signal_set_txpending(cd);

    // Load PIO program instructions into instruction memory
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(can2040_program_instructions); i++)
        pio_hw->instr_mem[i] = can2040_program_instructions[i];

    // Initialize each state machine
    pio_sync_setup(cd);
    pio_rx_setup(cd);
    pio_match_setup(cd);
    pio_tx_setup(cd);

    // Start state machines by enabling them
    pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
}

/**
 * @brief Initial setup of GPIO pins and PIO state machines
 *
 * @param cd Pointer to the can2040 structure
 * @param sys_clock System clock frequency
 * @param bitrate CANbus bitrate
 */
static void pio_setup(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate) {
    // Configure PIO0 or PIO1 clock based on pio_num
    uint32_t rb = cd->pio_num ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
    rp2040_clear_reset(rb);

    // Calculate and set clock divider for state machines
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t div = (256 / PIO_CLOCK_PER_BIT) * sys_clock / bitrate;
    int i;
    for (i = 0; i < 4; i++)
        pio_hw->sm[i].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;

    // Configure and start state machines
    pio_sm_setup(cd);

    // Map Rx/Tx GPIOs to PIO functions
    uint32_t pio_func = cd->pio_num ? 7 : 6;
    rp2040_gpio_peripheral(cd->gpio_rx, pio_func, 1);
    rp2040_gpio_peripheral(cd->gpio_tx, pio_func, 0);
}

/****************************************************************
 * CRC calculation
 ****************************************************************/

// Calculated 8-bit crc table (see scripts/crc.py)
static const uint16_t crc_table[256] = {
    0x0000,0x4599,0x4eab,0x0b32,0x58cf,0x1d56,0x1664,0x53fd,0x7407,0x319e,
    0x3aac,0x7f35,0x2cc8,0x6951,0x6263,0x27fa,0x2d97,0x680e,0x633c,0x26a5,
    0x7558,0x30c1,0x3bf3,0x7e6a,0x5990,0x1c09,0x173b,0x52a2,0x015f,0x44c6,
    0x4ff4,0x0a6d,0x5b2e,0x1eb7,0x1585,0x501c,0x03e1,0x4678,0x4d4a,0x08d3,
    0x2f29,0x6ab0,0x6182,0x241b,0x77e6,0x327f,0x394d,0x7cd4,0x76b9,0x3320,
    0x3812,0x7d8b,0x2e76,0x6bef,0x60dd,0x2544,0x02be,0x4727,0x4c15,0x098c,
    0x5a71,0x1fe8,0x14da,0x5143,0x73c5,0x365c,0x3d6e,0x78f7,0x2b0a,0x6e93,
    0x65a1,0x2038,0x07c2,0x425b,0x4969,0x0cf0,0x5f0d,0x1a94,0x11a6,0x543f,
    0x5e52,0x1bcb,0x10f9,0x5560,0x069d,0x4304,0x4836,0x0daf,0x2a55,0x6fcc,
    0x64fe,0x2167,0x729a,0x3703,0x3c31,0x79a8,0x28eb,0x6d72,0x6640,0x23d9,
    0x7024,0x35bd,0x3e8f,0x7b16,0x5cec,0x1975,0x1247,0x57de,0x0423,0x41ba,
    0x4a88,0x0f11,0x057c,0x40e5,0x4bd7,0x0e4e,0x5db3,0x182a,0x1318,0x5681,
    0x717b,0x34e2,0x3fd0,0x7a49,0x29b4,0x6c2d,0x671f,0x2286,0x2213,0x678a,
    0x6cb8,0x2921,0x7adc,0x3f45,0x3477,0x71ee,0x5614,0x138d,0x18bf,0x5d26,
    0x0edb,0x4b42,0x4070,0x05e9,0x0f84,0x4a1d,0x412f,0x04b6,0x574b,0x12d2,
    0x19e0,0x5c79,0x7b83,0x3e1a,0x3528,0x70b1,0x234c,0x66d5,0x6de7,0x287e,
    0x793d,0x3ca4,0x3796,0x720f,0x21f2,0x646b,0x6f59,0x2ac0,0x0d3a,0x48a3,
    0x4391,0x0608,0x55f5,0x106c,0x1b5e,0x5ec7,0x54aa,0x1133,0x1a01,0x5f98,
    0x0c65,0x49fc,0x42ce,0x0757,0x20ad,0x6534,0x6e06,0x2b9f,0x7862,0x3dfb,
    0x36c9,0x7350,0x51d6,0x144f,0x1f7d,0x5ae4,0x0919,0x4c80,0x47b2,0x022b,
    0x25d1,0x6048,0x6b7a,0x2ee3,0x7d1e,0x3887,0x33b5,0x762c,0x7c41,0x39d8,
    0x32ea,0x7773,0x248e,0x6117,0x6a25,0x2fbc,0x0846,0x4ddf,0x46ed,0x0374,
    0x5089,0x1510,0x1e22,0x5bbb,0x0af8,0x4f61,0x4453,0x01ca,0x5237,0x17ae,
    0x1c9c,0x5905,0x7eff,0x3b66,0x3054,0x75cd,0x2630,0x63a9,0x689b,0x2d02,
    0x276f,0x62f6,0x69c4,0x2c5d,0x7fa0,0x3a39,0x310b,0x7492,0x5368,0x16f1,
    0x1dc3,0x585a,0x0ba7,0x4e3e,0x450c,0x0095
};

/**
 * @brief Update a CRC with 8 bits of data
 *
 * @param crc Current CRC value
 * @param data New data byte to include in CRC
 * @return uint32_t Updated CRC value
 */
static uint32_t crc_byte(uint32_t crc, uint32_t data) {
    return (crc << 8) ^ crc_table[((crc >> 7) ^ data) & 0xFF];
}

/**
 * @brief Update a CRC with multiple bytes of data
 *
 * @param crc Current CRC value
 * @param data New data to include in CRC
 * @param num Number of bytes to process (1-4)
 * @return uint32_t Updated CRC value
 */
static inline uint32_t crc_bytes(uint32_t crc, uint32_t data, uint32_t num) {
    switch (num) {
    default:
        crc = crc_byte(crc, data >> 24); // FALLTHRU
    case 3:
        crc = crc_byte(crc, data >> 16); // FALLTHRU
    case 2:
        crc = crc_byte(crc, data >> 8);  // FALLTHRU
    case 1:
        crc = crc_byte(crc, data);
    }
    return crc;
}

/****************************************************************
 * Bit unstuffing
 ****************************************************************/

/**
 * @brief Add a specified number of bits from data to the bit unstuffer
 *
 * @param bu Pointer to the bitunstuffer structure
 * @param data Bits to add
 * @param count Number of bits to add
 */
static void unstuf_add_bits(struct can2040_bitunstuffer *bu, uint32_t data, uint32_t count) {
    uint32_t mask = (1 << count) - 1;
    bu->stuffed_bits = (bu->stuffed_bits << count) | (data & mask);
    bu->count_stuff = count;
}

/**
 * @brief Reset state and set the next desired number of unstuffed bits to extract
 *
 * @param bu Pointer to the bitunstuffer structure
 * @param num_bits Number of bits to extract
 */
static void unstuf_set_count(struct can2040_bitunstuffer *bu, uint32_t num_bits) {
    bu->unstuffed_bits = 0;
    bu->count_unstuff = num_bits;
}

/**
 * @brief Clear bitstuffing state (used after CRC field to avoid bitstuffing ACK field)
 *
 * @param bu Pointer to the bitunstuffer structure
 */
static void unstuf_clear_state(struct can2040_bitunstuffer *bu) {
    uint32_t lb = 1 << bu->count_stuff;
    bu->stuffed_bits = (bu->stuffed_bits & (lb - 1)) | (lb << 1);
}

/**
 * @brief Restore raw bitstuffing state (used to undo unstuf_clear_state())
 *
 * @param bu Pointer to the bitunstuffer structure
 * @param data Raw data to restore
 */
static void unstuf_restore_state(struct can2040_bitunstuffer *bu, uint32_t data) {
    uint32_t cs = bu->count_stuff;
    bu->stuffed_bits = (bu->stuffed_bits & ((1 << cs) - 1)) | (data << cs);
}

/**
 * @brief Pull bits from the unstuffer as specified in unstuf_set_count()
 *
 * @param bu Pointer to the bitunstuffer structure
 * @return int 0 if desired bits extracted, 1 if more data needed, -1/-2 for errors
 */
static int unstuf_pull_bits(struct can2040_bitunstuffer *bu) {
    uint32_t sb = bu->stuffed_bits, edges = sb ^ (sb >> 1);
    uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2), rm_bits = ~e4;
    uint32_t cs = bu->count_stuff, cu = bu->count_unstuff;

    if (!cs)
        // Need more data
        return 1;

    for (;;) {
        uint32_t try_cnt = cs > cu ? cu : cs;
        for (;;) {
            uint32_t try_mask = ((1 << try_cnt) - 1) << (cs + 1 - try_cnt);
            if (likely(!(rm_bits & try_mask))) {
                // No stuff bits in try_cnt bits - copy into unstuffed_bits
                bu->count_unstuff = cu = cu - try_cnt;
                bu->count_stuff = cs = cs - try_cnt;
                bu->unstuffed_bits |= ((sb >> cs) & ((1 << try_cnt) - 1)) << cu;
                if (!cu)
                    // Extracted desired bits
                    return 0;
                break;
            }
            bu->count_stuff = cs = cs - 1;
            if (rm_bits & (1 << (cs + 1))) {
                // High bit is a stuff bit
                if (unlikely(rm_bits & (1 << cs))) {
                    // Six consecutive bits - a bitstuff error
                    if (sb & (1 << cs))
                        return -1; // Invalid bitstuffing (dominant bit)
                    return -2; // Invalid bitstuffing (recessive bit)
                }
                break;
            }
            // High bit not a stuff bit - limit try_cnt and retry
            bu->count_unstuff = cu = cu - 1;
            bu->unstuffed_bits |= ((sb >> cs) & 1) << cu;
            try_cnt /= 2;
        }
        if (likely(!cs))
            // Need more data
            return 1;
    }
}

/**
 * @brief Return the most recent raw (still stuffed) bits
 *
 * @param bu Pointer to the bitunstuffer structure
 * @return uint32_t Raw stuffed bits
 */
static uint32_t unstuf_get_raw(struct can2040_bitunstuffer *bu) {
    return bu->stuffed_bits >> bu->count_stuff;
}

/****************************************************************
 * Bit stuffing
 ****************************************************************/

/**
 * @brief Perform bit stuffing on the provided bits
 *
 * @param pb Pointer to the bit buffer
 * @param num_bits Number of bits to stuff
 * @return uint32_t Total number of bits after stuffing
 */
static uint32_t bitstuff(uint32_t *pb, uint32_t num_bits) {
    uint32_t b = *pb, count = num_bits;
    for (;;) {
        uint32_t try_cnt = num_bits, edges = b ^ (b >> 1);
        uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2), add_bits = ~e4;
        for (;;) {
            uint32_t try_mask = ((1 << try_cnt) - 1) << (num_bits - try_cnt);
            if (!(add_bits & try_mask)) {
                // No stuff bits needed in try_cnt bits
                if (try_cnt >= num_bits)
                    goto done;
                num_bits -= try_cnt;
                try_cnt = (num_bits + 1) / 2;
                continue;
            }
            if (add_bits & (1 << (num_bits - 1))) {
                // A stuff bit must be inserted prior to the high bit
                uint32_t low_mask = (1 << num_bits) - 1, low = b & low_mask;
                uint32_t high = (b & ~(low_mask >> 1)) << 1;
                b = high ^ low ^ (1 << (num_bits - 1));
                count += 1;
                if (num_bits <= 4)
                    goto done;
                num_bits -= 4;
                break;
            }
            // High bit doesn't need a stuff bit - accept it, limit try_cnt, retry
            num_bits--;
            try_cnt /= 2;
        }
    }
done:
    *pb = b;
    return count;
}

/**
 * @brief Structure for building bit-stuffed transmit messages
 */
struct bitstuffer_s {
    uint32_t prev_stuffed; // Previous stuffed bits
    uint32_t bitpos;       // Current bit position
    uint32_t *buf;         // Buffer to store stuffed bits
};

/**
 * @brief Push raw bits into the stuffer without performing bit stuffing
 *
 * @param bs Pointer to the bitstuffer_s structure
 * @param data Bits to push
 * @param count Number of bits to push
 */
static void bs_pushraw(struct bitstuffer_s *bs, uint32_t data, uint32_t count) {
    uint32_t bitpos = bs->bitpos;
    uint32_t wp = bitpos / 32;
    uint32_t bitused = bitpos % 32;
    uint32_t bitavail = 32 - bitused;
    uint32_t *fb = &bs->buf[wp];
    if (bitavail >= count) {
        fb[0] |= data << (bitavail - count);
    } else {
        fb[0] |= data >> (count - bitavail);
        fb[1] |= data << (32 - (count - bitavail));
    }
    bs->bitpos = bitpos + count;
}

/**
 * @brief Push bits into the stuffer with bit stuffing
 *
 * @param bs Pointer to the bitstuffer_s structure
 * @param data Bits to push
 * @param count Number of bits to push
 */
static void bs_push(struct bitstuffer_s *bs, uint32_t data, uint32_t count) {
    data &= (1 << count) - 1;
    uint32_t stuf = (bs->prev_stuffed << count) | data;
    uint32_t newcount = bitstuff(&stuf, count);
    bs_pushraw(bs, stuf, newcount);
    bs->prev_stuffed = stuf;
}

/**
 * @brief Finalize the bit stuffer by padding the final word with high bits
 *
 * @param bs Pointer to the bitstuffer_s structure
 * @return uint32_t Number of words after finalization
 */
static uint32_t bs_finalize(struct bitstuffer_s *bs) {
    uint32_t bitpos = bs->bitpos;
    uint32_t words = DIV_ROUND_UP(bitpos, 32);
    uint32_t extra = words * 32 - bitpos;
    if (extra)
        bs->buf[words - 1] |= (1 << extra) - 1;
    return words;
}

/****************************************************************
 * Transmit state tracking
 ****************************************************************/

// Transmit states (stored in cd->tx_state)
enum {
    TS_IDLE = 0,       // No transmission in progress
    TS_QUEUED = 1,     // Transmission queued
    TS_ACKING_RX = 2,  // Acknowledgment being handled for received frame
    TS_CONFIRM_TX = 3  // Confirmation of transmitted frame
};

/**
 * @brief Calculate queue array position from a transmit index
 *
 * @param cd Pointer to the can2040 structure
 * @param pos Transmit index position
 * @return uint32_t Position in the transmit queue array
 */
static uint32_t tx_qpos(struct can2040 *cd, uint32_t pos) {
    return pos % ARRAY_SIZE(cd->tx_queue);
}

/**
 * @brief Queue the next message for transmission in the PIO
 *
 * @param cd Pointer to the can2040 structure
 * @return uint32_t IRQ bits that need to be set
 */
static uint32_t tx_schedule_transmit(struct can2040 *cd) {
    if (cd->tx_state == TS_QUEUED && !pio_tx_did_fail(cd))
        // Already queued or actively transmitting
        return 0;

    uint32_t tx_pull_pos = cd->tx_pull_pos;
    if (readl(&cd->tx_push_pos) == tx_pull_pos) {
        // No new messages to transmit
        cd->tx_state = TS_IDLE;
        pio_signal_clear_txpending(cd);
        __DMB();
        if (likely(readl(&cd->tx_push_pos) == tx_pull_pos))
            return SI_TXPENDING;
        // Raced with can2040_transmit() - msg is now available for transmit
        pio_signal_set_txpending(cd);
    }

    cd->tx_state = TS_QUEUED;
    cd->stats.tx_attempt++;
    struct can2040_transmit *qt = &cd->tx_queue[tx_qpos(cd, tx_pull_pos)];
    pio_tx_send(cd, qt->stuffed_data, qt->stuffed_words);
    return 0;
}

/**
 * @brief Setup PIO state for acknowledgment injection
 *
 * @param cd Pointer to the can2040 structure
 * @param match_key Bit sequence key for matching
 */
static void tx_inject_ack(struct can2040 *cd, uint32_t match_key) {
    cd->tx_state = TS_ACKING_RX;
    pio_tx_inject_ack(cd, match_key);
}

/**
 * @brief Check if the current parsed message is feedback from current transmit
 *
 * @param cd Pointer to the can2040 structure
 * @return int 1 if it's a self transmit, -1 on error, 0 otherwise
 */
static int tx_check_local_message(struct can2040 *cd) {
    if (cd->tx_state != TS_QUEUED)
        return 0;
    struct can2040_transmit *qt = &cd->tx_queue[tx_qpos(cd, cd->tx_pull_pos)];
    struct can2040_msg *pm = &cd->parse_msg, *tm = &qt->msg;
    if (tm->id == pm->id) {
        if (qt->crc != cd->parse_crc || tm->dlc != pm->dlc
            || tm->data32[0] != pm->data32[0] || tm->data32[1] != pm->data32[1])
            // Message with same ID that differs in content - an error
            return -1;
        // This is a self transmit
        cd->tx_state = TS_CONFIRM_TX;
        return 1;
    }
    return 0;
}

/****************************************************************
 * Notification callbacks
 ****************************************************************/

// Report state flags (stored in cd->report_state)
enum {
    RS_NEED_EOF_FLAG = 1 << 2,
    // States
    RS_IDLE = 0, RS_NEED_RX_ACK = 1, RS_NEED_TX_ACK = 2,
    RS_NEED_RX_EOF = RS_NEED_RX_ACK | RS_NEED_EOF_FLAG,
    RS_NEED_TX_EOF = RS_NEED_TX_ACK | RS_NEED_EOF_FLAG,
};

/**
 * @brief Report an error to the calling code via the callback interface
 *
 * @param cd Pointer to the can2040 structure
 * @param error_code Specific error code to report
 */
static void report_callback_error(struct can2040 *cd, uint32_t error_code) {
    struct can2040_msg msg = {};
    cd->rx_cb(cd, CAN2040_NOTIFY_ERROR | error_code, &msg);
}

/**
 * @brief Report a received message to the calling code via the callback interface
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_callback_rx_msg(struct can2040 *cd) {
    cd->stats.rx_total++;
    cd->rx_cb(cd, CAN2040_NOTIFY_RX, &cd->parse_msg);
}

/**
 * @brief Report a successfully transmitted message to the calling code via the callback interface
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_callback_tx_msg(struct can2040 *cd) {
    writel(&cd->tx_pull_pos, cd->tx_pull_pos + 1);
    cd->stats.tx_total++;
    cd->rx_cb(cd, CAN2040_NOTIFY_TX, &cd->parse_msg);
}

/**
 * @brief Handle end-of-frame (EOF) phase completion and report the message
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_handle_eof(struct can2040 *cd) {
    if (cd->report_state & RS_NEED_EOF_FLAG) { // RS_NEED_xX_EOF
        // Successfully processed a new message - report to calling code
        pio_sync_normal_start_signal(cd);
        if (cd->report_state == RS_NEED_TX_EOF)
            report_callback_tx_msg(cd);
        else
            report_callback_rx_msg(cd);
    }
    cd->report_state = RS_IDLE;
    pio_match_clear(cd);
}

/**
 * @brief Check if the message being processed is not part of a transmit operation
 *
 * @param cd Pointer to the can2040 structure
 * @return int Non-zero if not in TX, else zero
 */
static int report_is_not_in_tx(struct can2040 *cd) {
    return !(cd->report_state & RS_NEED_TX_ACK);
}

/**
 * @brief Notify that a new message start has been detected
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_note_message_start(struct can2040 *cd) {
    pio_irq_set(cd, SI_MAYTX);
}

/**
 * @brief Setup for acknowledgment injection or confirmation based on transmission state
 *
 * @param cd Pointer to the can2040 structure
 * @return int 0 on success, -1 on error
 */
static int report_note_crc_start(struct can2040 *cd) {
    int ret = tx_check_local_message(cd);
    if (ret) {
        if (ret < 0)
            return -1;
        // This is a self transmit - setup tx EOF "matched" signal
        cd->report_state = RS_NEED_TX_ACK;
        uint32_t bits = (cd->parse_crc_bits << 9) | 0x0FF;
        pio_match_check(cd, pio_match_calc_key(bits, cd->parse_crc_pos + 9));
        return 0;
    }

    // Setup for ack inject (after rx FIFOs fully drained)
    cd->report_state = RS_NEED_RX_ACK;
    pio_signal_set_txpending(cd);
    pio_irq_set(cd, SI_MAYTX | SI_TXPENDING);
    return 0;
}

/**
 * @brief Notify that CRC was successfully matched
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_note_crc_success(struct can2040 *cd) {
    if (cd->report_state == RS_NEED_TX_ACK)
        // Enable "matched" IRQ for fast back-to-back transmit scheduling
        pio_irq_set(cd, SI_MAYTX | SI_MATCHED);
}

/**
 * @brief Notify that an acknowledgment was successfully received
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_note_ack_success(struct can2040 *cd) {
    if (cd->report_state == RS_IDLE)
        // Got "matched" signal already
        return;
    // Transition RS_NEED_xX_ACK to RS_NEED_xX_EOF
    cd->report_state |= RS_NEED_EOF_FLAG;
}

/**
 * @brief Notify that an end-of-frame was successfully received
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_note_eof_success(struct can2040 *cd) {
    if (cd->report_state == RS_IDLE)
        // Got "matched" signal already
        return;
    report_handle_eof(cd);
    pio_irq_set(cd, SI_TXPENDING);
}

/**
 * @brief Notify that unexpected data is being discarded
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_note_discarding(struct can2040 *cd) {
    if (cd->report_state != RS_IDLE) {
        cd->report_state = RS_IDLE;
        pio_match_clear(cd);
    }
    pio_sync_slow_start_signal(cd);
    pio_irq_set(cd, SI_MAYTX | SI_TXPENDING);
}

/**
 * @brief Handle acknowledgment completion for a received message
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_line_ackdone(struct can2040 *cd) {
    // Setup "matched" IRQ for fast RX callbacks
    uint32_t bits = (cd->parse_crc_bits << 8) | 0x7f;
    pio_match_check(cd, pio_match_calc_key(bits, cd->parse_crc_pos + 8));
    // Schedule next transmit (so it is ready for next frame line arbitration)
    uint32_t check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, SI_MAYTX | SI_MATCHED | check_txpending);
}

/**
 * @brief Handle a "matched" IRQ indicating successful transmission
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_line_matched(struct can2040 *cd) {
    // A match event indicates an ack and EOF are present
    if (cd->report_state != RS_IDLE) {
        // Transition RS_NEED_xX_ACK to RS_NEED_xX_EOF (if not already there)
        cd->report_state |= RS_NEED_EOF_FLAG;
        report_handle_eof(cd);
    }
    // Implement fast back-to-back TX scheduling (if applicable)
    uint32_t check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, check_txpending);
}

/**
 * @brief Handle detection of idle bus state indicating possible transmission opportunity
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_line_maytx(struct can2040 *cd) {
    // Line is idle - may be unexpected EOF, missed ack injection, or missed "matched" signal
    if (cd->report_state != RS_IDLE)
        report_handle_eof(cd);
    uint32_t check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, check_txpending);
}

/**
 * @brief Handle a pending transmission request
 *
 * @param cd Pointer to the can2040 structure
 */
static void report_line_txpending(struct can2040 *cd) {
    uint32_t pio_irqs = pio_irq_get(cd);
    if (pio_irqs == (SI_MAYTX | SI_TXPENDING | SI_RX_DATA)
        && cd->report_state == RS_NEED_RX_ACK) {
        // Ack inject request from report_note_crc_start()
        uint32_t mk = pio_match_calc_key(cd->parse_crc_bits, cd->parse_crc_pos);
        tx_inject_ack(cd, mk);
        pio_irq_set(cd, SI_MAYTX | SI_ACKDONE);
        return;
    }
    // TX request from can2040_transmit(), report_note_eof_success(),
    // or report_note_discarding().
    uint32_t check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, (pio_irqs & ~SI_TXPENDING) | check_txpending);
}

/****************************************************************
 * Input state tracking
 ****************************************************************/

// Parsing states (stored in cd->parse_state)
enum {
    MS_START,       // Waiting for Start of Frame (SOF)
    MS_HEADER,      // Processing header
    MS_EXT_HEADER,  // Processing extended header
    MS_DATA0,       // Processing first part of data
    MS_DATA1,       // Processing second part of data
    MS_CRC,         // Processing CRC
    MS_ACK,         // Processing acknowledgment
    MS_EOF0,        // Processing first part of EOF
    MS_EOF1,        // Processing second part of EOF
    MS_DISCARD      // Discarding unexpected data
};

/**
 * @brief Reset any bits in the incoming parsing state
 *
 * @param cd Pointer to the can2040 structure
 */
static void data_state_clear_bits(struct can2040 *cd) {
    cd->raw_bit_count = 0;
    cd->unstuf.stuffed_bits = 0;
    cd->unstuf.count_stuff = 0;
}

/**
 * @brief Transition to the next parsing state
 *
 * @param cd Pointer to the can2040 structure
 * @param state Next state to transition to
 * @param num_bits Number of bits to await in the new state
 */
static void data_state_go_next(struct can2040 *cd, uint32_t state, uint32_t num_bits) {
    cd->parse_state = state;
    unstuf_set_count(&cd->unstuf, num_bits);
}

/**
 * @brief Transition to the discard state to drop all bits until a certain condition
 *
 * @param cd Pointer to the can2040 structure
 */
static void data_state_go_discard(struct can2040 *cd) {
    if (pio_rx_check_stall(cd)) {
        // CPU couldn't keep up for some read data - must reset PIO state
        data_state_clear_bits(cd);
        pio_sm_setup(cd);
        report_callback_error(cd, 0);
    }

    data_state_go_next(cd, MS_DISCARD, 32);

    // Clear report state and update hardware IRQs after transition to MS_DISCARD
    report_note_discarding(cd);
}

/**
 * @brief Note a data parse error and transition to discard state
 *
 * @param cd Pointer to the can2040 structure
 */
static void data_state_go_error(struct can2040 *cd) {
    cd->stats.parse_error++;
    data_state_go_discard(cd);
}

/**
 * @brief Handle reception of six dominant bits indicating a line error
 *
 * @param cd Pointer to the can2040 structure
 */
static void data_state_line_error(struct can2040 *cd) {
    if (cd->parse_state == MS_DISCARD)
        data_state_go_discard(cd);
    else
        data_state_go_error(cd);
}

/**
 * @brief Handle reception of six unexpected passive bits indicating potential bitstuffing error
 *
 * @param cd Pointer to the can2040 structure
 */
static void data_state_line_passive(struct can2040 *cd) {
    if (cd->parse_state != MS_DISCARD && cd->parse_state != MS_START) {
        // Bitstuff error
        data_state_go_error(cd);
        return;
    }

    uint32_t stuffed_bits = unstuf_get_raw(&cd->unstuf);
    uint32_t dom_bits = ~stuffed_bits;
    if (!dom_bits) {
        // Counter overflow in "sync" state machine - reset it
        data_state_clear_bits(cd);
        pio_sm_setup(cd);
        data_state_go_discard(cd);
        return;
    }

    // Look for SOF after 10 passive bits (most "PIO sync" will produce)
    if (!(dom_bits & 0x3FF)) {
        data_state_go_next(cd, MS_START, 1);
        return;
    }

    data_state_go_discard(cd);
}

/**
 * @brief Transition to MS_CRC state to await CRC bits
 *
 * @param cd Pointer to the can2040 structure
 */
static void data_state_go_crc(struct can2040 *cd) {
    cd->parse_crc &= 0x7FFF;

    // Calculate raw stuffed bits after CRC and CRC delimiter
    uint32_t crcstart_bitpos = cd->raw_bit_count - cd->unstuf.count_stuff - 1;
    uint32_t crc_bits = (unstuf_get_raw(&cd->unstuf) << 15) | cd->parse_crc;
    uint32_t crc_bitcount = bitstuff(&crc_bits, 15 + 1) - 1;
    cd->parse_crc_bits = (crc_bits << 1) | 0x01; // Add CRC delimiter
    cd->parse_crc_pos = crcstart_bitpos + crc_bitcount + 1;

    int ret = report_note_crc_start(cd);
    if (ret) {
        data_state_go_error(cd);
        return;
    }
    data_state_go_next(cd, MS_CRC, 16);
}

/**
 * @brief Transition to MS_DATA0 state to await data bits
 *
 * @param cd Pointer to the can2040 structure
 * @param id Message ID
 * @param data Header data
 */
static void data_state_go_data(struct can2040 *cd, uint32_t id, uint32_t data) {
    if (data & (0x03 << 4)) {
        // Not a supported header
        data_state_go_discard(cd);
        return;
    }
    cd->parse_msg.data32[0] = cd->parse_msg.data32[1] = 0;
    uint32_t dlc = data & 0x0F;
    cd->parse_msg.dlc = dlc;
    if (data & (1 << 6)) {
        // Remote Transmission Request (RTR) frame
        dlc = 0;
        id |= CAN2040_ID_RTR;
    }
    cd->parse_msg.id = id;
    if (dlc)
        data_state_go_next(cd, MS_DATA0, dlc >= 4 ? 32 : dlc * 8);
    else
        data_state_go_crc(cd);
}

/**
 * @brief Handle reception of first bit of header (after Start-of-Frame)
 *
 * @param cd Pointer to the can2040 structure
 * @param data Header ID bit
 */
static void data_state_update_start(struct can2040 *cd, uint32_t data) {
    cd->parse_msg.id = data;
    report_note_message_start(cd);
    data_state_go_next(cd, MS_HEADER, 17);
}

/**
 * @brief Handle reception of next 17 header bits
 *
 * @param cd Pointer to the can2040 structure
 * @param data Header bits
 */
static void data_state_update_header(struct can2040 *cd, uint32_t data) {
    data |= cd->parse_msg.id << 17;
    if ((data & 0x60) == 0x60) {
        // Extended header
        cd->parse_msg.id = data;
        data_state_go_next(cd, MS_EXT_HEADER, 20);
        return;
    }
    cd->parse_crc = crc_bytes(0, data, 3);
    data_state_go_data(cd, (data >> 7) & 0x7FF, data);
}

/**
 * @brief Handle reception of additional 20 bits of "extended header"
 *
 * @param cd Pointer to the can2040 structure
 * @param data Extended header bits
 */
static void data_state_update_ext_header(struct can2040 *cd, uint32_t data) {
    uint32_t hdr1 = cd->parse_msg.id;
    uint32_t crc = crc_bytes(0, hdr1 >> 4, 2);
    cd->parse_crc = crc_bytes(crc, ((hdr1 & 0x0f) << 20) | data, 3);
    uint32_t id = (((hdr1 << 11) & 0x1ffc0000) | ((hdr1 << 13) & 0x3e000)
                   | (data >> 7) | CAN2040_ID_EFF);
    data_state_go_data(cd, id, data);
}

/**
 * @brief Handle reception of first 1-4 bytes of data content
 *
 * @param cd Pointer to the can2040 structure
 * @param data Data bits
 */
static void data_state_update_data0(struct can2040 *cd, uint32_t data) {
    uint32_t dlc = cd->parse_msg.dlc, bits = dlc >= 4 ? 32 : dlc * 8;
    cd->parse_crc = crc_bytes(cd->parse_crc, data, dlc);
    cd->parse_msg.data32[0] = __builtin_bswap32(data << (32 - bits));
    if (dlc > 4)
        data_state_go_next(cd, MS_DATA1, dlc >= 8 ? 32 : (dlc - 4) * 8);
    else
        data_state_go_crc(cd);
}

/**
 * @brief Handle reception of bytes 5-8 of data content
 *
 * @param cd Pointer to the can2040 structure
 * @param data Data bits
 */
static void data_state_update_data1(struct can2040 *cd, uint32_t data) {
    uint32_t dlc = cd->parse_msg.dlc, bits = dlc >= 8 ? 32 : (dlc - 4) * 8;
    cd->parse_crc = crc_bytes(cd->parse_crc, data, dlc - 4);
    cd->parse_msg.data32[1] = __builtin_bswap32(data << (32 - bits));
    data_state_go_crc(cd);
}

/**
 * @brief Handle reception of 16 bits of message CRC (15 CRC bits + CRC delimiter)
 *
 * @param cd Pointer to the can2040 structure
 * @param data CRC bits
 */
static void data_state_update_crc(struct can2040 *cd, uint32_t data) {
    if (((cd->parse_crc << 1) | 1) != data) {
        data_state_go_error(cd);
        return;
    }

    report_note_crc_success(cd);
    unstuf_clear_state(&cd->unstuf);
    data_state_go_next(cd, MS_ACK, 2);
}

/**
 * @brief Handle reception of 2 bits of acknowledgment phase (ACK, ACK delimiter)
 *
 * @param cd Pointer to the can2040 structure
 * @param data ACK bits
 */
static void data_state_update_ack(struct can2040 *cd, uint32_t data) {
    if (data != 0x01) {
        // Undo unstuf_clear_state() for correct SOF detection in data_state_line_passive()
        unstuf_restore_state(&cd->unstuf, (cd->parse_crc_bits << 2) | data);

        data_state_go_error(cd);
        return;
    }
    report_note_ack_success(cd);
    data_state_go_next(cd, MS_EOF0, 4);
}

/**
 * @brief Handle reception of first four end-of-frame (EOF) bits
 *
 * @param cd Pointer to the can2040 structure
 * @param data EOF bits
 */
static void data_state_update_eof0(struct can2040 *cd, uint32_t data) {
    if (data != 0x0F || pio_rx_check_stall(cd)) {
        data_state_go_error(cd);
        return;
    }
    unstuf_clear_state(&cd->unstuf);
    data_state_go_next(cd, MS_EOF1, 5);
}

/**
 * @brief Handle reception of end-of-frame (EOF) bits 5-7 and first two Interframe Space (IFS) bits
 *
 * @param cd Pointer to the can2040 structure
 * @param data EOF and IFS bits
 */
static void data_state_update_eof1(struct can2040 *cd, uint32_t data) {
    if (data == 0x1F) {
        // Success
        report_note_eof_success(cd);
        data_state_go_next(cd, MS_START, 1);
    } else if (data >= 0x1C || (data >= 0x18 && report_is_not_in_tx(cd))) {
        // Message fully transmitted - followed by "overload frame"
        report_note_eof_success(cd);
        data_state_go_discard(cd);
    } else {
        data_state_go_error(cd);
    }
}

/**
 * @brief Handle data received while in MS_DISCARD state
 *
 * @param cd Pointer to the can2040 structure
 * @param data Discarded data bits
 */
static void data_state_update_discard(struct can2040 *cd, uint32_t data) {
    data_state_go_discard(cd);
}

/**
 * @brief Update parsing state after reading the bits of the current field
 *
 * @param cd Pointer to the can2040 structure
 * @param data Bits of the current field
 */
static void data_state_update(struct can2040 *cd, uint32_t data) {
    switch (cd->parse_state) {
    case MS_START:
        data_state_update_start(cd, data);
        break;
    case MS_HEADER:
        data_state_update_header(cd, data);
        break;
    case MS_EXT_HEADER:
        data_state_update_ext_header(cd, data);
        break;
    case MS_DATA0:
        data_state_update_data0(cd, data);
        break;
    case MS_DATA1:
        data_state_update_data1(cd, data);
        break;
    case MS_CRC:
        data_state_update_crc(cd, data);
        break;
    case MS_ACK:
        data_state_update_ack(cd, data);
        break;
    case MS_EOF0:
        data_state_update_eof0(cd, data);
        break;
    case MS_EOF1:
        data_state_update_eof1(cd, data);
        break;
    case MS_DISCARD:
        data_state_update_discard(cd, data);
        break;
    }
}

/****************************************************************
 * Input processing
 ****************************************************************/

/**
 * @brief Process incoming data from PIO "rx" state machine
 *
 * @param cd Pointer to the can2040 structure
 * @param rx_data Received data bits
 */
static void process_rx(struct can2040 *cd, uint32_t rx_data) {
    unstuf_add_bits(&cd->unstuf, rx_data, PIO_RX_WAKE_BITS);
    cd->raw_bit_count += PIO_RX_WAKE_BITS;

    // Undo bit stuffing
    for (;;) {
        int ret = unstuf_pull_bits(&cd->unstuf);
        if (likely(ret > 0)) {
            // Need more data
            break;
        } else if (likely(!ret)) {
            // Pulled the next field - process it
            data_state_update(cd, cd->unstuf.unstuffed_bits);
        } else {
            if (ret == -1)
                // 6 consecutive high bits
                data_state_line_passive(cd);
            else
                // 6 consecutive low bits
                data_state_line_error(cd);
        }
    }
}

/**
 * @brief Main API IRQ notification function to handle PIO interrupts
 *
 * @param cd Pointer to the can2040 structure
 */
void can2040_pio_irq_handler(struct can2040 *cd) {
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t ints = pio_hw->ints0;

    // Handle incoming RX data
    while (likely(ints & SI_RX_DATA)) {
        uint32_t rx_data = pio_hw->rxf[1];
        process_rx(cd, rx_data);
        ints = pio_hw->ints0;
        if (likely(!ints))
            return;
    }

    // Handle other IRQs based on their flags
    if (ints & SI_ACKDONE)
        // Ack of received message completed successfully
        report_line_ackdone(cd);
    else if (ints & SI_MATCHED)
        // Transmit message completed successfully
        report_line_matched(cd);
    else if (ints & SI_MAYTX)
        // Bus is idle, but not all bits may have been flushed yet
        report_line_maytx(cd);
    else if (ints & SI_TXPENDING)
        // Schedule a transmit
        report_line_txpending(cd);
}

/****************************************************************
 * Transmit queuing
 ****************************************************************/

/**
 * @brief API function to check if transmit space is available
 *
 * @param cd Pointer to the can2040 structure
 * @return int Non-zero if space is available, else zero
 */
int can2040_check_transmit(struct can2040 *cd) {
    uint32_t tx_pull_pos = readl(&cd->tx_pull_pos);
    uint32_t tx_push_pos = cd->tx_push_pos;
    uint32_t pending = tx_push_pos - tx_pull_pos;
    return pending < ARRAY_SIZE(cd->tx_queue);
}

/**
 * @brief API function to transmit a CAN message
 *
 * @param cd Pointer to the can2040 structure
 * @param msg Pointer to the CAN message to transmit
 * @return int 0 on success, -1 if transmit queue is full
 */
int can2040_transmit(struct can2040 *cd, struct can2040_msg *msg) {
    uint32_t tx_pull_pos = readl(&cd->tx_pull_pos);
    uint32_t tx_push_pos = cd->tx_push_pos;
    uint32_t pending = tx_push_pos - tx_pull_pos;
    if (pending >= ARRAY_SIZE(cd->tx_queue))
        // TX queue full
        return -1;

    // Copy message into transmit queue
    struct can2040_transmit *qt = &cd->tx_queue[tx_qpos(cd, tx_push_pos)];
    uint32_t id = msg->id;
    if (id & CAN2040_ID_EFF)
        qt->msg.id = id & ~0x20000000; // Clear EFF bit for internal use
    else
        qt->msg.id = id & (CAN2040_ID_RTR | 0x7FF); // Standard ID with RTR

    qt->msg.dlc = msg->dlc & 0x0F;
    uint32_t data_len = qt->msg.dlc > 8 ? 8 : qt->msg.dlc;
    if (qt->msg.id & CAN2040_ID_RTR)
        data_len = 0; // No data for RTR frames

    qt->msg.data32[0] = qt->msg.data32[1] = 0;
    memcpy(qt->msg.data, msg->data, data_len);

    // Calculate CRC and perform bit stuffing
    uint32_t crc = 0;
    memset(qt->stuffed_data, 0, sizeof(qt->stuffed_data));
    struct bitstuffer_s bs = { 1, 0, qt->stuffed_data };
    uint32_t edlc = qt->msg.dlc | (qt->msg.id & CAN2040_ID_RTR ? 0x40 : 0);
    if (qt->msg.id & CAN2040_ID_EFF) {
        // Extended header
        uint32_t id = qt->msg.id;
        uint32_t h1 = ((id & 0x1FFC0000) >> 11) | 0x60 | ((id & 0x003E0000) >> 13);
        uint32_t h2 = ((id & 0x1FFF) << 7) | edlc;
        crc = crc_bytes(crc, h1 >> 4, 2);
        crc = crc_bytes(crc, ((h1 & 0x0F) << 20) | h2, 3);
        bs_push(&bs, h1, 19);
        bs_push(&bs, h2, 20);
    } else {
        // Standard header
        uint32_t hdr = ((qt->msg.id & 0x7FF) << 7) | edlc;
        crc = crc_bytes(crc, hdr, 3);
        bs_push(&bs, hdr, 19);
    }
    uint32_t i;
    for (i = 0; i < data_len; i++) {
        uint32_t v = qt->msg.data[i];
        crc = crc_byte(crc, v);
        bs_push(&bs, v, 8);
    }
    qt->crc = crc & 0x7FFF;
    bs_push(&bs, qt->crc, 15);
    bs_pushraw(&bs, 1, 1); // CRC delimiter
    qt->stuffed_words = bs_finalize(&bs);

    // Submit the message by updating the push position
    writel(&cd->tx_push_pos, tx_push_pos + 1);

    // Wake up if in TS_IDLE state
    __DMB();
    pio_signal_set_txpending(cd);

    return 0;
}

/****************************************************************
 * Setup
 ****************************************************************/

/**
 * @brief API function to initialize can2040 code
 *
 * @param cd Pointer to the can2040 structure
 * @param pio_num PIO number to use (0 or 1)
 */
void can2040_setup(struct can2040 *cd, uint32_t pio_num) {
    memset(cd, 0, sizeof(*cd));
    cd->pio_num = !!pio_num;
    cd->pio_hw = cd->pio_num ? pio1_hw : pio0_hw;
}

/**
 * @brief API function to configure the callback function
 *
 * @param cd Pointer to the can2040 structure
 * @param rx_cb Callback function for receiving notifications
 */
void can2040_callback_config(struct can2040 *cd, can2040_rx_cb rx_cb) {
    cd->rx_cb = rx_cb;
}

/**
 * @brief API function to start the CANbus interface
 *
 * @param cd Pointer to the can2040 structure
 * @param sys_clock System clock frequency
 * @param bitrate CANbus bitrate
 * @param gpio_rx GPIO pin number for RX
 * @param gpio_tx GPIO pin number for TX
 */
void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate
                  , uint32_t gpio_rx, uint32_t gpio_tx) {
    cd->gpio_rx = gpio_rx;
    cd->gpio_tx = gpio_tx;
    data_state_clear_bits(cd);
    pio_setup(cd, sys_clock, bitrate);
    data_state_go_discard(cd);
}

/**
 * @brief API function to stop the can2040 code
 *
 * @param cd Pointer to the can2040 structure
 */
void can2040_stop(struct can2040 *cd) {
    pio_irq_disable(cd);
    pio_sm_setup(cd);
}

/**
 * @brief API function to access can2040 statistics
 *
 * @param cd Pointer to the can2040 structure
 * @param stats Pointer to the statistics structure to fill
 */
void can2040_get_statistics(struct can2040 *cd, struct can2040_stats *stats) {
    for (;;) {
        memcpy(stats, &cd->stats, sizeof(*stats));
        if (memcmp(stats, &cd->stats, sizeof(*stats)) == 0)
            // Successfully copied data
            return;
        // Raced with IRQ handler update - retry copy
    }
}
