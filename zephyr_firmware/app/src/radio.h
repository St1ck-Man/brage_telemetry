#ifndef RADIO_H
#define RADIO_H

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "sx128x/sx128x_hal.h"
#include "sx128x/sx128x.h"

/**
 * The radio layer bridges CAN and LoRa communication:
 *
 * TX Path (CAN → LoRa):
 * 1. Dequeue CAN messages from can_tx_queue via can_dequeue_tx_msg()
 * 2. Batch multiple CAN messages into a single protocol buffer
 * 3. Transmit when:
 *    - Batch timeout expires (10ms)
 *    - Max batch size reached (3-4 messages)
 *    - Buffer space exhausted
 * 4. Use protocol_begin_buffer() → protocol_append_can_frame() → protocol_finish_buffer()
 * 5. Call start_transmit() and wait for TX_DONE IRQ
 * 6. Return to RX mode
 *
 * RX Path (LoRa → CAN):
 * 1. Default state: listening via start_receive()
 * 2. On RX_DONE IRQ:
 *    - Call finish_receive() to get data
 *    - Validate with protocol_verify_and_extract()
 *    - Parse CAN messages with protocol_consume_msg()
 *    - Queue each message via can_queue_rx_msg()
 *    - Return to RX mode
 *
 * State Machine:
 * IDLE → RX (default state, listening)
 * RX → TX (when CAN data available)
 * TX → RX (after transmission complete)
 *
 * Threading:
 * - Radio TX thread: polls CAN queue, batches, transmits
 * - Radio RX IRQ handler: processes received packets
 * - Uses Zephyr work queues for IRQ handling
 *
 * Configuration:
 * - Max batch size: 3-4 messages (balance latency vs efficiency)
 * - Batch timeout: 10ms (acceptable latency for telemetry)
 * - LoRa buffer: 255 bytes max
 * - Protocol overhead: 6 bytes + 3 bytes per CAN message
 */

// Radio configuration
#define RADIO_MAX_BATCH_SIZE 4      // Max CAN messages per LoRa packet
#define RADIO_BATCH_TIMEOUT_MS 10
#define RADIO_BUFFER_SIZE 255

// Radio state
typedef enum {
    RADIO_STATE_IDLE,
    RADIO_STATE_RX,
    RADIO_STATE_TX
} radio_state_t;

/**
 * Initialize radio hardware and start RX/TX threads
 *
 * mod_params: LoRa modulation parameters (SF, BW, CR)
 * freq_hz: Radio frequency in Hz
 * returns: 0 on success, negative error code on failure
 */
int radio_init(sx128x_mod_params_lora_ranging_t mod_params, uint32_t freq_hz);

/**
 * Start radio transmission
 * What does a packet transmission look like?
 * 1. Write data to radio buffer (sx128x_write_buffer())
 * 2. Trigger RF transmission (sx128x_set_tx())
 * 3. Wait for radio to finish transmission (TX_DONE IRQ)
 * 4. Return to receive mode (sx128x_set_rx()) if there is no more data to send
 */
sx128x_hal_status_t start_transmit(uint8_t* data, size_t len);

sx128x_hal_status_t start_receive(uint8_t timeout, sx128x_irq_mask_t irq_flags, size_t len);

sx128x_hal_status_t finish_transmit(void);

/**
 * Finish reception and retrieve data
 * Called from RX_DONE IRQ handler
 *
 * buffer: Buffer to store received data
 * param: buffer_size Size of buffer
 * param: bytes_received Output: number of bytes received
 * returns: sx128x_hal_status_t
 */
sx128x_hal_status_t finish_receive(uint8_t *buffer, size_t buffer_size, size_t *bytes_received);

#endif /* RADIO_H */
