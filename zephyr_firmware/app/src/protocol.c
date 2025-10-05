#include "protocol.h"
#include <string.h>
#include <zephyr/sys/crc.h>

/* Protocol constants */
#define PROTOCOL_HEADER_MAGIC_0 'b'
#define PROTOCOL_HEADER_MAGIC_1 'r'
#define PROTOCOL_HEADER_SIZE 4 /* "br" + id + payload_length */
#define PROTOCOL_CRC_SIZE 2
#define PROTOCOL_MIN_PACKET_SIZE (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC_SIZE)

/* Helper function to write uint16_t in little-endian */
static inline void write_u16_le(uint8_t *buf, uint16_t value) {
  buf[0] = value & 0xFF;
  buf[1] = (value >> 8) & 0xFF;
}

/* Helper function to read uint16_t in little-endian */
static inline uint16_t read_u16_le(const uint8_t *buf) {
  return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

int protocol_begin_buffer(uint8_t *data, size_t data_len, uint8_t id) {
  if (!data || data_len < PROTOCOL_MIN_PACKET_SIZE) {
    return -1;
  }

  /* Write header: "br" + id + payload_length (placeholder 0) */
  data[0] = PROTOCOL_HEADER_MAGIC_0;
  data[1] = PROTOCOL_HEADER_MAGIC_1;
  data[2] = id;
  data[3] = 0; /* Payload length placeholder, will be updated in finish */

  return PROTOCOL_HEADER_SIZE;
}

int protocol_append_can_frame(uint8_t *data, size_t data_len, size_t index,
                              const CanMsg *msg) {
  if (!data || !msg || msg->length > 64) {
    return -1;
  }

  /* CAN frame: id(2 bytes LE) + length(1 byte) + data[length] */
  size_t frame_size = 1 + 2 + 1 + msg->length;

  /* Check if we have enough space (including CRC at the end) */
  if (index + frame_size + PROTOCOL_CRC_SIZE > data_len) {
    return -1;
  }

  size_t idx = index;
  write_u16_le(&data[idx], msg->id);
  idx += 2;
  data[idx++] = msg->length;
  memcpy(&data[idx], msg->data, msg->length);
  idx += msg->length;

  return idx;
}

int protocol_finish_buffer(uint8_t *data, size_t index) {
  if (!data || index < PROTOCOL_HEADER_SIZE) {
    return -1;
  }

  /* Calculate payload length */
  size_t payload_len = index - PROTOCOL_HEADER_SIZE;
  if (payload_len > 255) {
    return -1; /* Payload length must fit in 1 byte */
  }

  /* Update payload length in header */
  data[3] = (uint8_t)payload_len;

  /* Calculate CRC16-IBM-SDLC over header + payload */
  uint16_t crc = crc16_itu_t(0xFFFF, data, index);

  /* Append CRC in little-endian */
  write_u16_le(&data[index], crc);

  return index + PROTOCOL_CRC_SIZE;
}

int protocol_verify_and_extract(const uint8_t *data, size_t len,
                                const uint8_t **payload_out,
                                size_t *payload_len_out) {
  if (!data || !payload_out || !payload_len_out ||
      len < PROTOCOL_MIN_PACKET_SIZE) {
    return -1;
  }

  /* Verify magic header */
  if (data[0] != PROTOCOL_HEADER_MAGIC_0 ||
      data[1] != PROTOCOL_HEADER_MAGIC_1) {
    return -1;
  }

  /* Extract payload length from header */
  uint8_t payload_len = data[3];

  /* Verify total packet size matches */
  size_t expected_len = PROTOCOL_HEADER_SIZE + payload_len + PROTOCOL_CRC_SIZE;
  if (len != expected_len) {
    return -1;
  }

  /* Verify CRC */
  size_t crc_offset = PROTOCOL_HEADER_SIZE + payload_len;
  uint16_t received_crc = read_u16_le(&data[crc_offset]);
  uint16_t calculated_crc = crc16_itu_t(0xFFFF, data, crc_offset);

  if (received_crc != calculated_crc) {
    return -1;
  }

  /* Extract payload */
  *payload_out = &data[PROTOCOL_HEADER_SIZE];
  *payload_len_out = payload_len;

  return 0;
}

int protocol_consume_msg(const uint8_t *data, size_t len, CanMsg *msg_out,
                         const uint8_t **remaining_out,
                         size_t *remaining_len_out) {
  if (!data || !msg_out || !remaining_out || !remaining_len_out || len == 0) {
    return -1;
  }

  size_t idx = 1;

  /* CAN frame: 'c' + id(2 bytes LE) + length(1 byte) + data[length] */
  if (len < 1 + 2 + 1) {
    return -1; /* Not enough data for CAN frame header */
  }

  uint16_t can_id = read_u16_le(&data[idx]);
  idx += 2;

  uint8_t can_len = data[idx];
  idx += 1;

  if (can_len > 64) {
    return -1; /* Invalid CAN length */
  }

  if (len < idx + can_len) {
    return -1; /* Not enough data for CAN payload */
  }

  /* Fill message structure */
  msg_out->id = can_id;
  msg_out->length = can_len;
  memcpy(msg_out->data, &data[idx], can_len);
  idx += can_len;

  /* Set remaining data pointer and length */
  *remaining_out = &data[idx];
  *remaining_len_out = len - idx;

  return 0;
}
