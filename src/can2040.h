#ifndef _CAN2040_H
#define _CAN2040_H

#include <stdint.h> // uint32_t

struct can2040_msg {
    uint32_t addr;
    uint32_t data_len;
    union {
        uint8_t d1[8];
        uint32_t d4[2];
    };
};

enum {
    CAN2040_ID_RX = 1<<20,
    CAN2040_ID_TX = 1<<21,
    CAN2040_ID_TX_FAIL = 1<<22,
    CAN2040_ID_ERROR = 1<<23,
};
struct can2040;
typedef void (*can2040_rx_cb)(struct can2040 *cd, uint32_t id
                              , struct can2040_msg *msg);

void can2040_setup(struct can2040 *cd, uint32_t pio_num);
void can2040_callback_config(struct can2040 *cd, can2040_rx_cb rx_cb);
void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate
                   , uint32_t gpio_rx, uint32_t gpio_tx);
void can2040_shutdown(struct can2040 *cd);
void can2040_pio_irq_handler(struct can2040 *cd);
int can2040_check_transmit(struct can2040 *cd);
int can2040_transmit(struct can2040 *cd, struct can2040_msg msg);


/****************************************************************
 * Internal definitions
 ****************************************************************/

struct can2040_bitunstuffer {
    uint32_t stuffed_bits, count_stuff;
    uint32_t unstuffed_bits, count_unstuff;
};

struct can2040_transmit {
    struct can2040_msg msg;
    uint32_t crc, stuffed_words, stuffed_data[5];
};

struct can2040 {
    // Setup
    uint32_t pio_num;
    void *pio_hw;
    uint32_t gpio_rx, gpio_tx;
    can2040_rx_cb rx_cb;

    // Input data state
    uint32_t parse_state;
    uint32_t parse_hdr, parse_crc, parse_datapos;
    struct can2040_msg parse_msg;

    // Bit unstuffing
    uint8_t latest_rx;
    struct can2040_bitunstuffer unstuf;
    uint32_t raw_bit_count;

    // Transmits
    uint32_t cancel_count;
    uint32_t in_transmit;
    uint32_t tx_pull_pos, tx_push_pos;
    struct can2040_transmit tx_queue[4];
};

#endif // can2040.h
