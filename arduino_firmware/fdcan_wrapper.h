#ifndef FDCAN_WRAPPER_H
#define FDCAN_WRAPPER_H

#include <Arduino.h>

#ifdef STM32C0xx
#include "stm32c0xx_hal_fdcan.h"
#endif

// CAN message structure compatible with your code
typedef struct __attribute__((packed)) {
  uint16_t id;         // 11-bit CAN identifier (0-0x7FF)
  uint8_t data_length; // CAN data length (0-8)
  uint8_t data[8];     // 8 byte data payload
} CAN_message_t;

class FDCAN_Wrapper {
private:
  FDCAN_HandleTypeDef hfdcan;
  FDCAN_TxHeaderTypeDef txHeader;
  FDCAN_RxHeaderTypeDef rxHeader;
  bool initialized;

public:
  FDCAN_Wrapper() : initialized(false) {}

  void begin() {
    // Configure FDCAN peripheral
    hfdcan.Instance = FDCAN1;
    hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan.Init.AutoRetransmission = ENABLE;
    hfdcan.Init.TransmitPause = DISABLE;
    hfdcan.Init.ProtocolException = ENABLE;

    // Nominal bit timing for 500 kbps (assuming 80 MHz FDCAN clock)
    // Adjust these values based on your actual clock configuration
    hfdcan.Init.NominalPrescaler = 10;
    hfdcan.Init.NominalSyncJumpWidth = 1;
    hfdcan.Init.NominalTimeSeg1 = 13;
    hfdcan.Init.NominalTimeSeg2 = 2;

    // Data bit timing (not used in classic CAN mode)
    hfdcan.Init.DataPrescaler = 1;
    hfdcan.Init.DataSyncJumpWidth = 1;
    hfdcan.Init.DataTimeSeg1 = 1;
    hfdcan.Init.DataTimeSeg2 = 1;

    // Message RAM configuration
    hfdcan.Init.StdFiltersNbr = 1;
    hfdcan.Init.ExtFiltersNbr = 0;
    hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    // Initialize FDCAN
    if (HAL_FDCAN_Init(&hfdcan) != HAL_OK) {
      initialized = false;
      return;
    }

    // Configure global filter to accept all messages
    FDCAN_FilterTypeDef filterConfig;
    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = 0x0;
    filterConfig.FilterID2 = 0x0; // Accept all

    if (HAL_FDCAN_ConfigFilter(&hfdcan, &filterConfig) != HAL_OK) {
      initialized = false;
      return;
    }

    // Configure global filter to reject all non-matching frames
    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
      initialized = false;
      return;
    }

    // Start FDCAN
    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) {
      initialized = false;
      return;
    }

    initialized = true;
  }

  void setBaudRate(uint32_t baudrate) {
    // Baudrate is set in begin(), could reconfigure here if needed
    // For now, this is a no-op as we're using 500kbps
  }

  bool write(uint16_t id, uint8_t *data, uint8_t length) {
    if (!initialized || length > 8) {
      return false;
    }

    // Configure TX header
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = length << 16; // Convert to FDCAN DLC format
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    // Add message to TX FIFO
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &txHeader, data) != HAL_OK) {
      return false;
    }

    return true;
  }

  int available() {
    if (!initialized) {
      return 0;
    }

    // Check if messages are available in RX FIFO 0
    uint32_t fillLevel = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0);
    return fillLevel > 0 ? 1 : 0;
  }

  bool read(CAN_message_t &msg) {
    if (!initialized) {
      return false;
    }

    // Check if messages are available
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0) {
      return false;
    }

    // Receive message
    if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &rxHeader, msg.data) !=
        HAL_OK) {
      return false;
    }

    // Extract message info
    msg.id = rxHeader.Identifier;
    msg.data_length =
        (rxHeader.DataLength >> 16); // Convert from FDCAN DLC format

    return true;
  }
};

#endif // FDCAN_WRAPPER_H
