// This is a simple example program using can2040 and the PICO SDK.
//
// See the CMakeLists.txt file for information on compiling.

#include <pico/stdlib.h>
#include <stdio.h>

#include "../src/can2040.h"

// Simple example of irq safe queue (this is not multi-core safe)
#define QUEUE_SIZE 128 // Must be power of 2
static struct {
    uint32_t pull_pos;
    volatile uint32_t push_pos;
    struct can2040_msg queue[QUEUE_SIZE];
} MessageQueue;

// Internal storage for can2040 module
static struct can2040 cbus;

// Main canbus callback (called from irq handler)
static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        // Example message filter
        uint32_t id = msg->id;
        if (id < 0x101 || id > 0x201)
            return;

        // Add to queue
        uint32_t push_pos = MessageQueue.push_pos;
        uint32_t pull_pos = MessageQueue.pull_pos;
        if (push_pos + 1 == pull_pos)
            // No space in queue
            return;
        MessageQueue.queue[push_pos % QUEUE_SIZE] = *msg;
        MessageQueue.push_pos = push_pos + 1;
    }
}

// PIO interrupt handler
static void
PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

// Initialize the can2040 module
void
canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = SYS_CLK_HZ, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

int
main(void)
{
    stdio_init_all();
    canbus_setup();

    // Main loop
    for (;;) {
        uint32_t push_pos = MessageQueue.push_pos;
        uint32_t pull_pos = MessageQueue.pull_pos;
        if (push_pos == pull_pos)
            // No new messages read.
            continue;

        // Pop message from local receive queue
        struct can2040_msg *qmsg = &MessageQueue.queue[pull_pos % QUEUE_SIZE];
        struct can2040_msg msg = *qmsg;
        MessageQueue.pull_pos++;

        // Report message found on local receive queue
        printf("msg: id=0x%x dlc=%d data=%02x%02x%02x%02x%02x%02x%02x%02x\n",
               msg.id, msg.dlc, msg.data[0], msg.data[1], msg.data[2],
               msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

        // Demo of message transmit
        if (msg.id == 0x101) {
            struct can2040_msg tmsg;
            tmsg.id = 0x102;
            tmsg.dlc = 8;
            tmsg.data32[0] = 0xabcd;
            tmsg.data32[1] = msg.data32[0];
            int sts = can2040_transmit(&cbus, &tmsg);
            printf("Sent message (status=%d)\n", sts);
        }
    }

    return 0;
}
