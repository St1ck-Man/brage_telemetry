# CAN-LoRa Bridge Testing Guide

## System Overview

This firmware implements a **bidirectional CAN-LoRa bridge** that:
- Receives CAN messages and forwards them over LoRa
- Receives LoRa messages and forwards them to CAN bus
- Provides manual testing via button press
- Shows visual feedback with LED indicators

---

## Hardware Setup

### Required Equipment
- **2x STM32 Nucleo boards** (with FDCAN support)
- **2x SX1280 LoRa modules**
- **CAN transceiver modules** (e.g., MCP2551, TJA1050)
- **CAN bus wiring** (CAN_H, CAN_L, 120Ω termination resistors)
- **USB cables** for serial monitoring and power

### Connections

#### Board 1 (Transmitter/Receiver)
```
STM32 Pin    │ Connection
─────────────┼────────────────
PA11         │ CAN RX (to transceiver)
PA12         │ CAN TX (to transceiver)
PC13         │ USER_BTN (built-in)
LED_GREEN    │ TX indicator
LED_BLUE     │ RX indicator
```

#### Board 2 (Transmitter/Receiver)
```
Same as Board 1
```

#### CAN Bus Wiring
```
Board 1 CAN_H ──┬──── Board 2 CAN_H
                │
               120Ω (termination)
                │
Board 1 CAN_L ──┴──── Board 2 CAN_L
```

---

## Basic Testing Scenarios

### Test 1: Button-Triggered CAN-to-LoRa-to-CAN Loop

**Objective:** Verify complete message path: CAN → LoRa → CAN

**Steps:**
1. Connect both boards to CAN bus
2. Open serial monitors on both boards (115200 baud)
3. Press USER_BTN on **Board 1**

**Expected Behavior:**
- **Board 1 Serial:**
  ```
  *** BUTTON PRESSED - Sending Test CAN Message ***
  [CAN] Sending message | ID: 0x100 | Length: 8
  [CAN] Message sent successfully
  ```

- **Board 2 Serial:**
  ```
  === CAN MESSAGE RECEIVED ===
  [CAN RX] ID: 0x100 | Length: 8
  [BRIDGE] Forwarding CAN message over LoRa...
  [TX] Starting transmission of 11 bytes
  ```

- **Board 1 Serial (receiving LoRa):**
  ```
  === LoRa PACKET RECEIVED ===
  [LoRa RX] RSSI: -45 dBm
  [BRIDGE] Valid CAN message detected, forwarding to CAN bus...
  [BRIDGE] Successfully forwarded to CAN bus
  ```

- **Board 2 Serial (receiving CAN again):**
  ```
  === CAN MESSAGE RECEIVED ===
  [CAN RX] ID: 0x100 | Length: 8
  (Loop continues...)
  ```

**LED Behavior:**
- Board 1: BLUE flashes (CAN RX), GREEN flashes (LoRa TX)
- Board 2: BLUE flashes (LoRa RX → CAN TX)

**Success Criteria:**
✓ Message appears on both serial monitors
✓ LEDs flash on both boards
✓ RSSI and SNR values are reasonable (>-100 dBm)
✓ Packet counter increments

---

### Test 2: Continuous Button Testing

**Objective:** Stress test the bridge with multiple rapid transmissions

**Steps:**
1. Press USER_BTN on Board 1 multiple times (5-10 times)
2. Monitor both serial outputs
3. Count successful transmissions

**Expected Behavior:**
- Each button press generates unique CAN message (counter increments)
- All messages forwarded successfully over LoRa
- No packet loss or corruption
- LED indicators respond to each message

**Success Criteria:**
✓ Packet counter increments correctly
✓ No CRC errors reported
✓ No transmission failures
✓ Message data integrity maintained

---

### Test 3: Bidirectional Testing

**Objective:** Verify both boards can send and receive

**Steps:**
1. Press USER_BTN on **Board 1**
2. Wait for message to complete full loop
3. Press USER_BTN on **Board 2**
4. Compare packet counters

**Expected Behavior:**
- Both boards can originate CAN messages
- Both boards receive and forward LoRa packets
- Packet counters are independent per board

**Success Criteria:**
✓ Both directions work independently
✓ No collisions or interference
✓ Messages don't interfere with each other

---

### Test 4: Range Testing

**Objective:** Determine maximum LoRa range

**Steps:**
1. Keep Board 1 stationary
2. Move Board 2 progressively farther away
3. Press button periodically and monitor RSSI/SNR
4. Record distance when packets start failing

**Expected Behavior:**
- RSSI decreases with distance
- SNR decreases with distance
- Communication works until ~-120 dBm RSSI

**Success Criteria:**
✓ RSSI values correlate with distance
✓ Range meets datasheet expectations for SX1280
✓ Graceful degradation (CRC errors before complete loss)

---

### Test 5: External CAN Device Integration

**Objective:** Test with real CAN device (ECU, sensor, etc.)

**Steps:**
1. Connect external CAN device to bus
2. Configure device to send periodic messages
3. Monitor serial output on both STM32 boards

**Expected Behavior:**
- External CAN messages automatically captured
- Messages forwarded over LoRa to other board
- Other board forwards to its CAN bus
- All CAN devices see all messages

**Success Criteria:**
✓ External CAN messages properly decoded
✓ CAN ID and data preserved through bridge
✓ Timing acceptable for application

---

## Advanced Testing Scenarios

### Test 6: High-Frequency CAN Traffic

**Setup:**
- Configure external CAN device for 100 msg/sec
- Monitor both boards

**Metrics to Observe:**
- Packet loss percentage
- Average latency (CAN RX → LoRa TX → CAN TX)
- Buffer overflow conditions

### Test 7: Message Filtering

**Modify Code:**
```cpp
// In handle_can_messages(), add filter:
if (msg.id >= 0x100 && msg.id <= 0x1FF) {
    // Only forward specific ID range
}
```

**Test:**
- Send various CAN IDs
- Verify only allowed IDs forwarded

### Test 8: Error Injection

**Tests:**
- Disconnect CAN bus mid-transmission
- Move boards out of LoRa range
- Send malformed CAN messages

**Expected:**
- Graceful error handling
- Appropriate error messages on serial
- System recovers when issue resolved

---

## Debugging Tips

### No CAN Messages Received
- Check CAN baud rate (default: 500 kbps)
- Verify CAN_H and CAN_L not swapped
- Ensure 120Ω termination resistors present
- Check CAN transceiver power supply

### No LoRa Communication
- Verify both radios initialized successfully
- Check antenna connections
- Ensure same LoRa settings on both boards
- Reduce distance for initial testing

### Intermittent Packet Loss
- Check for RF interference
- Verify power supply stability
- Add delays between transmissions
- Check for CAN bus errors

### Serial Monitor Issues
- Ensure 115200 baud rate
- Enable "Both NL & CR" line ending
- Try different USB port/cable
- Check board reset/power cycle

---

## Performance Metrics

### Expected Values
| Metric                  | Typical Value       |
|-------------------------|---------------------|
| CAN-to-LoRa latency     | 10-50 ms           |
| LoRa-to-CAN latency     | 10-50 ms           |
| Max throughput          | ~50 msg/sec        |
| LoRa range (line of sight) | 500m - 2km     |
| Packet success rate     | >95%               |

---

## Improvements & Suggestions

### Software Enhancements

1. **Message Queue**
   - Implement FIFO buffer for CAN messages
   - Prevents packet loss during burst traffic
   ```cpp
   #define CAN_QUEUE_SIZE 32
   CAN_message_t canQueue[CAN_QUEUE_SIZE];
   ```

2. **Acknowledgment System**
   - Add ACK packets for critical messages
   - Retransmission on timeout
   ```cpp
   struct AckMessage {
       uint8_t type;  // ACK/NACK
       uint16_t msgId;
   };
   ```

3. **Message Timestamping**
   - Add millisecond timestamp to each message
   - Measure end-to-end latency
   ```cpp
   struct CAN_message_stamped {
       uint32_t timestamp;
       CAN_message_t msg;
   };
   ```

4. **Diagnostic Statistics**
   - Track messages sent/received/failed
   - Calculate packet loss rate
   ```cpp
   struct Stats {
       uint32_t canRx, canTx;
       uint32_t loraRx, loraTx;
       uint32_t errors;
   };
   ```

5. **Multiple CAN Filters**
   - Hardware CAN filters for specific IDs
   - Reduce CPU load
   ```cpp
   can.setFilter(0, 0x100, 0x7F0);  // Accept 0x100-0x10F
   ```

### Hardware Improvements

1. **CAN Bus Isolation**
   - Use isolated CAN transceivers (ISO1050)
   - Protect microcontroller from bus faults

2. **Better Antennas**
   - Use high-gain directional antennas
   - Increase range significantly

3. **Power Management**
   - Add sleep mode when idle
   - Battery-powered operation

4. **Status Display**
   - Add OLED display
   - Show real-time stats without serial

### Testing Infrastructure

1. **Automated Test Suite**
   - Python script to send/verify messages
   - Continuous integration testing

2. **CAN Bus Simulator**
   - Generate realistic CAN traffic
   - Test under various load conditions

3. **Log Data to SD Card**
   - Record all messages for analysis
   - Debug intermittent issues

4. **Over-the-Air Updates**
   - Update firmware via LoRa
   - Remote deployment

---

## Troubleshooting Matrix

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| "CAN initialization failed" | Wrong pins, no transceiver | Check wiring, power |
| "LoRa initialization failed" | SX1280 connection issue | Check SPI wiring |
| Button does nothing | Wrong pin or pull-up | Verify USER_BTN = PC13 |
| CAN works, LoRa doesn't | Radio config mismatch | Ensure same settings |
| LoRa works, CAN doesn't | Bus termination missing | Add 120Ω resistors |
| Random crashes | Stack overflow | Reduce buffer sizes |

---

## Contact & Support

For issues or improvements, check:
- RadioLib documentation: https://github.com/jgromes/RadioLib
- STM32_CAN library: https://github.com/nopnop2002/Arduino-STM32-CAN
- STM32 Reference Manual for your specific MCU

---

**Last Updated:** 2025-10-05
