#ifndef CAN_H
#define CAN_H

#include "protocol.h"
#include <zephyr/kernel.h>

/**
 * Initialize CAN peripheral and start RX/TX threads
 *
 * Configures CAN-FD with:
 * - Nominal bitrate: 500 kbps
 * - Data bitrate: 1 Mbps
 * - FD mode enabled
 * - Max data length: 64 bytes
 *
 * @return 0 on success, negative error code on failure
 */
int can_init(void);

/**
 * Queue a received CAN message from radio for transmission on CAN bus
 *
 * This function is called by the radio layer when a CAN message
 * needs to be transmitted on the CAN bus.
 *
 * @param msg CAN message to queue for transmission
 * @return 0 on success, -ENOMEM if queue is full
 */
int can_queue_rx_msg(const CanMsg *msg);

/**
 * Dequeue a CAN message received from bus for radio transmission
 *
 * This function is called by the radio layer to get CAN messages
 * that were received from the CAN bus and need to be transmitted
 * over the radio.
 *
 * @param msg Buffer to store the dequeued message
 * @param timeout Timeout for waiting if queue is empty
 * @return 0 on success, -EAGAIN if timeout, negative error code on failure
 */
int can_dequeue_tx_msg(CanMsg *msg, k_timeout_t timeout);

#endif /* CAN_H */
