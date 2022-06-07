// Software CANbus implementation for rp2040
//
// Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include <string.h> // memset
#include "RP2040.h" // hw_set_bits
#include "can2040.h" // can2040_setup
#include "hardware/regs/dreq.h" // DREQ_PIO0_RX1
#include "hardware/structs/dma.h" // dma_hw
#include "hardware/structs/iobank0.h" // iobank0_hw
#include "hardware/structs/padsbank0.h" // padsbank0_hw
#include "hardware/structs/pio.h" // pio0_hw
#include "hardware/structs/resets.h" // RESETS_RESET_PIO0_BITS


/****************************************************************
 * rp2040 and low-level helper functions
 ****************************************************************/

// Helper compiler definitions
#define barrier() __asm__ __volatile__("": : :"memory")
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

// Helper functions for writing to "io" memory
static inline void writel(void *addr, uint32_t val) {
    barrier();
    *(volatile uint32_t *)addr = val;
}
static inline uint32_t readl(const void *addr) {
    uint32_t val = *(volatile const uint32_t *)addr;
    barrier();
    return val;
}

// rp2040 helper function to clear a hardware reset bit
static void
rp2040_clear_reset(uint32_t reset_bit)
{
    if (resets_hw->reset & reset_bit) {
        resets_hw->reset &= reset_bit;
        hw_clear_bits(&resets_hw->reset, reset_bit);
        while (!(resets_hw->reset_done & reset_bit))
            ;
    }
}

// Helper to set the mode and extended function of a pin
static void
rp2040_gpio_peripheral(uint32_t gpio, int func, int pull_up)
{
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

#define PIO_CLOCK_PER_BIT 32

#define can2040_offset_sync_found_end_of_message 2u
#define can2040_offset_sync_signal_start 4u
#define can2040_offset_sync_entry 6u
#define can2040_offset_sync_end 13u
#define can2040_offset_shared_rx_read 13u
#define can2040_offset_shared_rx_end 15u
#define can2040_offset_ack_no_match 18u
#define can2040_offset_ack_end 25u
#define can2040_offset_tx_got_recessive 25u
#define can2040_offset_tx_start 26u
#define can2040_offset_tx_error_loop 30u

static const uint16_t can2040_program_instructions[] = {
    0x0085, //  0: jmp    y--, 5
    0x0048, //  1: jmp    x--, 8
    0xe13a, //  2: set    x, 26                  [1]
    0x00cc, //  3: jmp    pin, 12
    0xc000, //  4: irq    nowait 0
    0x00c0, //  5: jmp    pin, 0
    0xc040, //  6: irq    clear 0
    0xe228, //  7: set    x, 8                   [2]
    0xf242, //  8: set    y, 2                   [18]
    0xc104, //  9: irq    nowait 4               [1]
    0x03c5, // 10: jmp    pin, 5                 [3]
    0x0307, // 11: jmp    7                      [3]
    0x0043, // 12: jmp    x--, 3
    0x20c4, // 13: wait   1 irq, 4
    0x4001, // 14: in     pins, 1
    0xa046, // 15: mov    y, isr
    0x00b2, // 16: jmp    x != y, 18
    0xc002, // 17: irq    nowait 2
    0x40eb, // 18: in     osr, 11
    0x4054, // 19: in     y, 20
    0xa047, // 20: mov    y, osr
    0x8080, // 21: pull   noblock
    0xa027, // 22: mov    x, osr
    0x0098, // 23: jmp    y--, 24
    0xa0e2, // 24: mov    osr, y
    0xa242, // 25: nop                           [2]
    0x6021, // 26: out    x, 1
    0xa001, // 27: mov    pins, x
    0x20c4, // 28: wait   1 irq, 4
    0x00d9, // 29: jmp    pin, 25
    0x023a, // 30: jmp    !x, 26                 [2]
};

static void
pio_sync_setup(struct can2040 *cd)
{
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
    pio_hw->txf[0] = PIO_CLOCK_PER_BIT / 2 * 8 - 5 - 1;
    sm->instr = 0x80a0; // pull block
    sm->instr = can2040_offset_sync_entry; // jmp sync_entry
}

static void
pio_rx_setup(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[1];
    sm->execctrl = (
        (can2040_offset_shared_rx_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = cd->gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
                     | 8 << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB
                     | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    sm->instr = can2040_offset_shared_rx_read; // jmp shared_rx_read
}

static void
pio_ack_setup(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[2];
    sm->execctrl = (
        (can2040_offset_ack_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = cd->gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0;
    sm->instr = 0xe040; // set y, 0
    sm->instr = 0xa0e2; // mov osr, y
    sm->instr = 0xa02a, // mov x, !y
    sm->instr = can2040_offset_ack_no_match; // jmp ack_no_match
}

static void
pio_tx_setup(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->execctrl = (
        cd->gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB
        | can2040_offset_tx_error_loop << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_tx_error_loop << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    sm->pinctrl = (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
                   | 1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB
                   | cd->gpio_tx << PIO_SM0_PINCTRL_SET_BASE_LSB
                   | cd->gpio_tx << PIO_SM0_PINCTRL_OUT_BASE_LSB);
    sm->instr = 0xe001; // set pins, 1
    sm->instr = 0xe081; // set pindirs, 1
}

static void
pio_tx_reset(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
    pio_hw->irq = (1 << 2) | (1<< 3); // clear irq 2 and 3
    if (pio_hw->flevel & PIO_FLEVEL_TX3_BITS) {
        struct pio_sm_hw *sm = &pio_hw->sm[3];
        sm->shiftctrl = 0;
        sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                         | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    }
}

static void
pio_tx_send(struct can2040 *cd, uint32_t *data, uint32_t count)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_tx_reset(cd);
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0xa242; // nop [2]
    int i;
    for (i=0; i<count; i++)
        pio_hw->txf[3] = data[i];
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c0; // wait 1 irq, 0
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;
}

static void
pio_tx_cancel(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = 0xe001; // set pins, 1
}

static void
pio_ack_check(struct can2040 *cd, uint32_t crc_bits, uint32_t rx_bit_pos)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t key = (crc_bits & 0x1fffff) | ((-rx_bit_pos) << 21);
    pio_hw->txf[2] = key;

    // Raise irq after ack completes
    pio_hw->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM2_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

static void
pio_ack_inject(struct can2040 *cd, uint32_t crc_bits, uint32_t rx_bit_pos)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_tx_reset(cd);
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0xc023; // irq wait 3
    pio_hw->txf[3] = 0x7fffffff;
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c2; // wait 1 irq, 2
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;

    uint32_t key = (crc_bits & 0x1fffff) | ((-rx_bit_pos) << 21);
    pio_hw->txf[2] = key;

    // Raise irq after ack completes
    pio_hw->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM3_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

static void
pio_ack_cancel(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->txf[2] = 0;
}

static int
pio_rx_check_stall(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    return pio_hw->fdebug & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1));
}

static void
pio_sync_enable_idle_irq(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

static void
pio_sync_disable_idle_irq(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->inte0 = PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

static void
pio_sync_normal_start_signal(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    pio_hw->instr_mem[eom_idx] = 0xe13a; // set x, 26 [1]
}

static void
pio_sync_slow_start_signal(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    pio_hw->instr_mem[eom_idx] = 0xa127; // mov x, osr [1]
}

static void
pio_sm_setup(struct can2040 *cd)
{
    // Reset state machines
    pio_hw_t *pio_hw = cd->pio_hw;
    pio_hw->ctrl = PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS;
    pio_hw->fdebug = 0xffffffff;

    // Load pio program
    int i;
    for (i=0; i<ARRAY_SIZE(can2040_program_instructions); i++)
        pio_hw->instr_mem[i] = can2040_program_instructions[i];

    // Set initial state machine state
    pio_sync_setup(cd);
    pio_rx_setup(cd);
    pio_ack_setup(cd);
    pio_tx_setup(cd);

    // Start state machines
    pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
}

#define PIO_FUNC 6

static void
pio_setup(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate)
{
    // Configure pio0 clock
    uint32_t rb = cd->pio_num ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
    rp2040_clear_reset(rb);

    // Setup and sync pio state machine clocks
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t div = (256 / PIO_CLOCK_PER_BIT) * sys_clock / bitrate;
    int i;
    for (i=0; i<4; i++)
        pio_hw->sm[i].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;

    // Configure state machines
    pio_sm_setup(cd);

    // Map Rx/Tx gpios
    rp2040_gpio_peripheral(cd->gpio_rx, PIO_FUNC, 1);
    rp2040_gpio_peripheral(cd->gpio_tx, PIO_FUNC, 0);
}


/****************************************************************
 * CRC calculation
 ****************************************************************/

static uint32_t
crcbits(uint32_t crc, uint32_t data, uint32_t count)
{
    int i;
    for (i=count-1; i>=0; i--) {
        uint32_t bit = (data >> i) & 1;
        crc = ((crc >> 14) & 1) ^ bit ? (crc << 1) ^ 0x4599 : (crc << 1);
    }
    return crc;
}


/****************************************************************
 * Bit unstuffing
 ****************************************************************/

static void
unstuf_add_bits(struct can2040_bitunstuffer *bu, uint32_t data, uint32_t count)
{
    uint32_t mask = (1 << count) - 1;
    bu->stuffed_bits = (bu->stuffed_bits << count) | (data & mask);
    bu->count_stuff = count;
}

static void
unstuf_set_count(struct can2040_bitunstuffer *bu, uint32_t count)
{
    bu->unstuffed_bits = 0;
    bu->count_unstuff = count;
}

static void
unstuf_clear_state(struct can2040_bitunstuffer *bu)
{
    uint32_t sb = bu->stuffed_bits, edges = sb ^ (sb >> 1);
    uint32_t cs = bu->count_stuff, re = edges >> cs;
    if (!(re & 1) && (re & 0xf))
        bu->stuffed_bits ^= 1 << cs;
}

static int
unstuf_pull_bits(struct can2040_bitunstuffer *bu)
{
    uint32_t sb = bu->stuffed_bits, edges = sb ^ (sb >> 1);
    uint32_t ub = bu->unstuffed_bits;
    uint32_t cs = bu->count_stuff, cu = bu->count_unstuff;
    for (;;) {
        if (!cu) {
            // Extracted desired bits
            bu->unstuffed_bits = ub;
            bu->count_stuff = cs;
            bu->count_unstuff = cu;
            return 0;
        }
        if (!cs) {
            // Need more data
            bu->unstuffed_bits = ub;
            bu->count_stuff = cs;
            bu->count_unstuff = cu;
            return 1;
        }
        cs--;
        if ((edges >> (cs+1)) & 0xf) {
            // Normal data
            cu--;
            ub |= ((sb >> cs) & 1) << cu;
        } else if (((edges >> cs) & 0x1f) == 0x00) {
            // Six consecutive bits - a bitstuff error
            bu->unstuffed_bits = ub;
            bu->count_stuff = cs;
            bu->count_unstuff = cu;
            if ((sb >> cs) & 1)
                return -1;
            return -2;
        }
    }
}


/****************************************************************
 * Bit stuffing
 ****************************************************************/

static uint32_t
bitstuff(uint32_t *pb, uint32_t num_bits)
{
    uint32_t b = *pb, edges = b ^ (b >> 1), count = num_bits;
    int i;
    for (i=num_bits-1; i>=0; i--) {
        if (!((edges >> i) & 0xf)) {
            uint32_t mask = (1 << (i + 1)) - 1;
            uint32_t low = b & mask, high = (b & ~(mask >> 1)) << 1;
            b = high ^ low ^ (1 << i);
            i -= 3;
            count++;
            edges = b ^ (b >> 1);
        }
    }
    *pb = b;
    return count;
}

struct bitstuffer_s {
    uint32_t prev_stuffed, bitpos, *buf, crc;
};

static void
bs_pushraw(struct bitstuffer_s *bs, uint32_t data, uint32_t count)
{
    uint32_t bitpos = bs->bitpos;
    uint32_t wp = bitpos / 32, bitused = bitpos % 32, bitavail = 32 - bitused;
    uint32_t *fb = &bs->buf[wp];
    if (bitavail >= count) {
        fb[0] |= data << (bitavail - count);
    } else {
        fb[0] |= data >> (count - bitavail);
        fb[1] |= data << (32 - (count - bitavail));
    }
    bs->bitpos = bitpos + count;
}

static void
bs_push(struct bitstuffer_s *bs, uint32_t data, uint32_t count)
{
    data &= (1 << count) - 1;
    bs->crc = crcbits(bs->crc, data, count);
    uint32_t stuf = (bs->prev_stuffed << count) | data;
    uint32_t newcount = bitstuff(&stuf, count);
    bs_pushraw(bs, stuf, newcount);
    bs->prev_stuffed = stuf;
}

static uint32_t
bs_finalize(struct bitstuffer_s *bs)
{
    uint32_t bitpos = bs->bitpos;
    uint32_t words = DIV_ROUND_UP(bitpos, 32);
    uint32_t extra = words * 32 - bitpos;
    if (extra)
        bs->buf[words - 1] |= (1 << extra) - 1;
    return words;
}


/****************************************************************
 * Notification callbacks
 ****************************************************************/

static void
report_error(struct can2040 *cd, uint32_t error_code)
{
    struct can2040_msg msg = {};
    cd->rx_cb(cd, CAN2040_NOTIFY_ERROR | error_code, &msg);
}

static void
report_rx_msg(struct can2040 *cd)
{
    cd->rx_cb(cd, CAN2040_NOTIFY_RX, &cd->parse_msg);
}

static void
report_tx_msg(struct can2040 *cd, struct can2040_msg *msg)
{
    cd->rx_cb(cd, CAN2040_NOTIFY_TX, msg);
}

static void
report_tx_fail(struct can2040 *cd, struct can2040_msg *msg)
{
    cd->rx_cb(cd, CAN2040_NOTIFY_TX_FAIL, msg);
}


/****************************************************************
 * Transmit
 ****************************************************************/

static uint32_t
tx_qpos(struct can2040 *cd, uint32_t pos)
{
    return pos % ARRAY_SIZE(cd->tx_queue);
}

static void
tx_do_schedule(struct can2040 *cd)
{
    if (cd->in_transmit || cd->tx_push_pos == cd->tx_pull_pos)
        return;
    if (cd->cancel_count > 32) { // XXX
        cd->cancel_count = 0;
        uint32_t tx_pull_pos = cd->tx_pull_pos;
        cd->tx_pull_pos++;
        report_tx_fail(cd, &cd->tx_queue[tx_qpos(cd, tx_pull_pos)].msg);
        if (cd->tx_push_pos == cd->tx_pull_pos)
            return;
    }
    cd->in_transmit = 1;
    struct can2040_transmit *qt = &cd->tx_queue[tx_qpos(cd, cd->tx_pull_pos)];
    pio_tx_send(cd, qt->stuffed_data, qt->stuffed_words);
}

static void
tx_cancel(struct can2040 *cd)
{
    if (cd->in_transmit) {
        cd->in_transmit = 0;
        cd->cancel_count++;
    }
    pio_tx_cancel(cd);
}

static int
tx_check_self_transmit(struct can2040 *cd)
{
    if (!cd->in_transmit)
        return 0;
    struct can2040_transmit *qt = &cd->tx_queue[tx_qpos(cd, cd->tx_pull_pos)];
    struct can2040_msg *pm = &cd->parse_msg;
    if (qt->crc == cd->parse_crc
        && qt->msg.id == pm->id && qt->msg.dlc == pm->dlc
        && qt->msg.d4[0] == pm->d4[0] && qt->msg.d4[1] == pm->d4[1]) {
        return 1;
    }
    tx_cancel(cd);
    return 0;
}

static void
tx_finalize(struct can2040 *cd)
{
    tx_cancel(cd);
    uint32_t tx_pull_pos = cd->tx_pull_pos;
    cd->tx_pull_pos++;
    report_tx_msg(cd, &cd->tx_queue[tx_qpos(cd, tx_pull_pos)].msg);
}


/****************************************************************
 * Input state tracking
 ****************************************************************/

enum {
    MS_START, MS_EXT_HEADER, MS_DATA, MS_CRC, MS_ACK, MS_EOF, MS_DISCARD
};

static void
data_state_report_frame(struct can2040 *cd)
{
    pio_sync_enable_idle_irq(cd);
    if (! cd->notify_pending)
        return;

    cd->notify_pending = 0;
    if (tx_check_self_transmit(cd))
        tx_finalize(cd);
    else
        report_rx_msg(cd);

    cd->cancel_count = 0;
    pio_sync_normal_start_signal(cd);
    tx_do_schedule(cd);
}

static void
data_state_go_discard(struct can2040 *cd)
{
    cd->parse_state = MS_DISCARD;
    cd->notify_pending = 0;
    unstuf_set_count(&cd->unstuf, 8);
    tx_cancel(cd);
    pio_sync_slow_start_signal(cd);
    pio_sync_enable_idle_irq(cd);
}

static void
data_state_go_error(struct can2040 *cd)
{
    data_state_go_discard(cd);
}

static void
data_state_go_idle(struct can2040 *cd)
{
    if (cd->parse_state == MS_START) {
        if (!cd->unstuf.count_stuff && cd->unstuf.stuffed_bits == 0xffffffff) {
            // Counter overflow in "sync" state machine - reset it
            pio_sync_setup(cd);
            cd->unstuf.stuffed_bits = 0;
            data_state_go_discard(cd);
            return;
        }
        unstuf_set_count(&cd->unstuf, 18);
        return;
    }
    if (cd->parse_state != MS_EOF)
        pio_sync_slow_start_signal(cd);
    pio_sync_disable_idle_irq(cd);
    pio_ack_cancel(cd);
    tx_do_schedule(cd);
    cd->parse_state = MS_START;
    cd->notify_pending = 0;
    unstuf_set_count(&cd->unstuf, 18);
}

static void
data_state_go_crc(struct can2040 *cd)
{
    cd->parse_state = MS_CRC;
    unstuf_set_count(&cd->unstuf, 15);
    cd->parse_crc &= 0x7fff;

    // Setup for ack injection (if receiving) or ack confirmation (if transmit)
    uint32_t cs = cd->unstuf.count_stuff;
    uint32_t crcstart_bitpos = cd->raw_bit_count - cs - 1;
    uint32_t last = (cd->unstuf.stuffed_bits >> cs) << 15;
    last |= cd->parse_crc;
    int crc_bitcount = bitstuff(&last, 15 + 1) - 1;

    if (tx_check_self_transmit(cd)) {
        last = (last << 3) | 0x05;
        pio_ack_check(cd, last, crcstart_bitpos + crc_bitcount + 3);
    } else {
        last = (last << 1) | 0x01;
        pio_ack_inject(cd, last, crcstart_bitpos + crc_bitcount + 1);
    }
    cd->notify_pending = 1;
}

static void
data_state_go_data(struct can2040 *cd, uint32_t id, uint32_t data)
{
    if (data & (0x03 << 4)) {
        // Not a supported header
        data_state_go_discard(cd);
        return;
    }
    cd->parse_msg.d4[0] = cd->parse_msg.d4[1] = 0;
    cd->parse_datapos = 0;
    cd->parse_msg.dlc = data & 0x0f;
    uint32_t data_len = CAN2040_DATA_LEN(cd->parse_msg);
    if (data & (1 << 6)) {
        data_len = 0;
        id |= CAN2040_ID_RTR;
    }
    cd->parse_msg.id = id;
    if (data_len) {
        cd->parse_state = MS_DATA;
        unstuf_set_count(&cd->unstuf, 8);
    } else {
        data_state_go_crc(cd);
    }
}

static void
data_state_update_start(struct can2040 *cd, uint32_t data)
{
    pio_sync_enable_idle_irq(cd);
    cd->parse_crc = crcbits(0, data, 18);
    if ((data & 0x60) == 0x60) {
        // Extended header
        cd->parse_msg.id = data;
        cd->parse_state = MS_EXT_HEADER;
        unstuf_set_count(&cd->unstuf, 20);
        return;
    }
    data_state_go_data(cd, (data >> 7) & 0x7ff, data);
}

static void
data_state_update_ext_header(struct can2040 *cd, uint32_t data)
{
    cd->parse_crc = crcbits(cd->parse_crc, data, 20);
    uint32_t hdr1 = cd->parse_msg.id;
    uint32_t id = (((hdr1 >> 7) & 0x7ff) | ((hdr1 & 0x1f) << 24)
                   | ((data >> 7) << 11) | CAN2040_ID_EFF);
    data_state_go_data(cd, id, data);
}

static void
data_state_update_data(struct can2040 *cd, uint32_t data)
{
    cd->parse_crc = crcbits(cd->parse_crc, data, 8);
    cd->parse_msg.d1[cd->parse_datapos++] = data;
    if (cd->parse_datapos >= CAN2040_DATA_LEN(cd->parse_msg)) {
        data_state_go_crc(cd);
    } else {
        unstuf_set_count(&cd->unstuf, 8);
    }
}

static void
data_state_update_crc(struct can2040 *cd, uint32_t data)
{
    if (cd->parse_crc != data) {
        pio_ack_cancel(cd);
        data_state_go_discard(cd);
        return;
    }

    cd->parse_state = MS_ACK;
    unstuf_clear_state(&cd->unstuf);
    unstuf_set_count(&cd->unstuf, 3);
}

static void
data_state_update_ack(struct can2040 *cd, uint32_t data)
{
    pio_ack_cancel(cd);
    if (data != 0x05) {
        data_state_go_discard(cd);

        // If cpu couldn't keep up for some read data then reset the pio state
        if (pio_rx_check_stall(cd)) {
            pio_sm_setup(cd);
            report_error(cd, 0);
        }
        return;
    }
    data_state_report_frame(cd);
    cd->parse_state = MS_EOF;
    unstuf_set_count(&cd->unstuf, 6);
}

static void
data_state_update_eof(struct can2040 *cd, uint32_t data)
{
    // The end-of-frame should have raised a bitstuff condition..
    data_state_go_discard(cd);
}

static void
data_state_update_discard(struct can2040 *cd, uint32_t data)
{
    data_state_go_discard(cd);
}

static void
data_state_update(struct can2040 *cd, uint32_t data)
{
    switch (cd->parse_state) {
    case MS_START: data_state_update_start(cd, data); break;
    case MS_EXT_HEADER: data_state_update_ext_header(cd, data); break;
    case MS_DATA: data_state_update_data(cd, data); break;
    case MS_CRC: data_state_update_crc(cd, data); break;
    case MS_ACK: data_state_update_ack(cd, data); break;
    case MS_EOF: data_state_update_eof(cd, data); break;
    case MS_DISCARD: data_state_update_discard(cd, data); break;
    }
}


/****************************************************************
 * Input processing
 ****************************************************************/

static void
process_rx(struct can2040 *cd, uint32_t rx_byte)
{
    unstuf_add_bits(&cd->unstuf, rx_byte, 8);
    cd->raw_bit_count += 8;

    // undo bit stuffing
    for (;;) {
        int ret = unstuf_pull_bits(&cd->unstuf);
        if (!ret) {
            // Pulled the next field - process it
            data_state_update(cd, cd->unstuf.unstuffed_bits);
        } else if (ret > 0) {
            // Need more data
            break;
        } else {
            if (ret == -1) {
                // 6 consecutive high bits
                data_state_go_idle(cd);
            } else {
                // 6 consecutive low bits
                data_state_go_error(cd);
            }
        }
    }
}

void
can2040_pio_irq_handler(struct can2040 *cd)
{
    pio_hw_t *pio_hw = cd->pio_hw;
    uint32_t ints = pio_hw->ints0;
    while (ints & PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS) {
        uint8_t rx_byte = pio_hw->rxf[1];
        process_rx(cd, rx_byte);
        ints = pio_hw->ints0;
    }

    if (ints & PIO_IRQ0_INTE_SM0_BITS)
        // Bus is idle, but not all bits flushed yet - force idle state
        data_state_go_idle(cd);
    else if (ints & (PIO_IRQ0_INTE_SM2_BITS|PIO_IRQ0_INTE_SM3_BITS))
        // Ack phase completed successfully
        data_state_report_frame(cd);
}


/****************************************************************
 * Transmit queuing
 ****************************************************************/

int
can2040_check_transmit(struct can2040 *cd)
{
    uint32_t tx_pull_pos = readl(&cd->tx_pull_pos);
    uint32_t tx_push_pos = cd->tx_push_pos;
    uint32_t pending = tx_push_pos - tx_pull_pos;
    return pending < ARRAY_SIZE(cd->tx_queue);
}

int
can2040_transmit(struct can2040 *cd, struct can2040_msg msg)
{
    uint32_t tx_pull_pos = readl(&cd->tx_pull_pos);
    uint32_t tx_push_pos = cd->tx_push_pos;
    uint32_t pending = tx_push_pos - tx_pull_pos;
    if (pending >= ARRAY_SIZE(cd->tx_queue))
        // Tx queue full
        return -1;

    // Copy msg into qt->msg
    struct can2040_transmit *qt = &cd->tx_queue[tx_qpos(cd, tx_push_pos)];
    if (msg.id & CAN2040_ID_EFF)
        qt->msg.id = msg.id & ~0x20000000;
    else
        qt->msg.id = msg.id & (CAN2040_ID_RTR | 0x7ff);
    qt->msg.dlc = msg.dlc & 0x0f;
    uint32_t data_len = CAN2040_DATA_LEN(qt->msg);
    if (qt->msg.id & CAN2040_ID_RTR)
        data_len = 0;
    qt->msg.d4[0] = qt->msg.d4[1] = 0;
    memcpy(qt->msg.d1, msg.d1, data_len);

    // Calculate crc and stuff bits
    memset(qt->stuffed_data, 0, sizeof(qt->stuffed_data));
    struct bitstuffer_s bs = { 1, 0, qt->stuffed_data, 0 };
    bs_push(&bs, qt->msg.id & 0x7ff, 12);
    if (qt->msg.id & CAN2040_ID_EFF)
        bs_push(&bs, ((qt->msg.id >> 11) & 0x3ffff) | 0xc0000, 20);
    bs_push(&bs, qt->msg.dlc | (qt->msg.id & CAN2040_ID_RTR ? 0x40 : 0), 7);
    int i;
    for (i=0; i<data_len; i++)
        bs_push(&bs, qt->msg.d1[i], 8);
    qt->crc = bs.crc & 0x7fff;
    bs_push(&bs, qt->crc, 15);
    bs_pushraw(&bs, 1, 1);
    qt->stuffed_words = bs_finalize(&bs);

    // Submit
    writel(&cd->tx_push_pos, tx_push_pos + 1);

    // Kick transmitter
    __disable_irq();
    if (cd->parse_state == MS_START)
        // XXX - not a good way to start tx
        tx_do_schedule(cd);
    __enable_irq();

    return 0;
}


/****************************************************************
 * Setup
 ****************************************************************/

void
can2040_setup(struct can2040 *cd, uint32_t pio_num)
{
    memset(cd, 0, sizeof(*cd));
    cd->pio_num = !!pio_num;
    cd->pio_hw = cd->pio_num ? pio1_hw : pio0_hw;
}

void
can2040_callback_config(struct can2040 *cd, can2040_rx_cb rx_cb)
{
    cd->rx_cb = rx_cb;
}

void
can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate
              , uint32_t gpio_rx, uint32_t gpio_tx)
{
    cd->gpio_rx = gpio_rx;
    cd->gpio_tx = gpio_tx;
    pio_setup(cd, sys_clock, bitrate);
    data_state_go_discard(cd);
}

void
can2040_shutdown(struct can2040 *cd)
{
    // XXX
}
