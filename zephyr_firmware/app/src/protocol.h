#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/**
 * CAN message structure
 */
typedef struct {
  uint16_t id;
  uint8_t length;
  uint8_t data[64];
} CanMsg;

/**
 * Begin a new protocol buffer with header
 *
 * @param data Buffer to write to
 * @param data_len Total buffer size
 * @param id Message ID (0-255)
 * @return Index after header, or -1 on error
 */
int protocol_begin_buffer(uint8_t *data, size_t data_len, uint8_t id);

/**
 * Append a CAN frame to the protocol buffer
 *
 * @param data Buffer to write to
 * @param data_len Total buffer size
 * @param index Current index in buffer
 * @param msg CAN message to append
 * @return New index after frame, or -1 on error
 */
int protocol_append_can_frame(uint8_t *data, size_t data_len, size_t index,
                              const CanMsg *msg);

/**
 * Finish the protocol buffer by adding CRC footer
 *
 * @param data Buffer containing the message
 * @param index Current index (end of payload)
 * @return Total packet length including CRC, or -1 on error
 */
int protocol_finish_buffer(uint8_t *data, size_t index);

/**
 * Verify header and CRC, extract payload
 *
 * @param data Received packet data
 * @param len Packet length
 * @param payload_out Pointer to set to payload start
 * @param payload_len_out Pointer to store payload length
 * @return 0 on success, -1 on error
 */
int protocol_verify_and_extract(const uint8_t *data, size_t len,
                                const uint8_t **payload_out,
                                size_t *payload_len_out);

/**
 * Parse next message from payload
 *
 * @param data Payload data
 * @param len Payload length
 * @param msg_out Message structure to fill
 * @param remaining_out Pointer to remaining data after this message
 * @param remaining_len_out Length of remaining data
 * @return 0 on success, -1 on error
 */
int protocol_consume_msg(const uint8_t *data, size_t len, CanMsg *msg_out,
                         const uint8_t **remaining_out,
                         size_t *remaining_len_out);

#endif /* PROTOCOL_H */
