#include "pin_config.h"
#include <RadioLib.h>
#include "fdcan_wrapper.h"

// CAN Configuration
FDCAN_Wrapper can;

// Global state variables
volatile bool isTransmitting = false;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50ms debounce
uint32_t packetCounter = 0;  // For unique packet identification

SX1280 radio = new Module(SX_CS, SX_DIO1, SX_RESET, SX_BUSY);

int transmissionState = RADIOLIB_ERR_NONE;

volatile bool receivedFlag = false;
volatile bool transmittedFlag = false;

void setRxFlag(void) { receivedFlag = true; }
void setTxFlag(void) { transmittedFlag = true; }

void handle_error(int state) {
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}
void init_can() {
  Serial.print(F("[CAN] Initializing ... "));

  // Initialize CAN bus at 500kbps
  can.begin();
  can.setBaudRate(500000); // 500 kbps
  Serial.println(F("success!"));
}

void init_radio()
{
  /* initializes sx1280 instance and starts receiving */

  Serial.print(F("[SX1280] Initializing ... "));
  int state = radio.begin();
  handle_error(state);

  radio.setPacketSentAction(setTxFlag);
  radio.setPacketReceivedAction(setRxFlag);

  start_receive();
} 

void start_receive()
{
  digitalWrite(RX_EN, HIGH);
  int state = radio.startReceive();
  handle_error(state);
}

/*Send an array of bytes (a packet)*/
int start_transmit(uint8_t* packet, size_t length) {
    if (packet == nullptr || length == 0) {
        Serial.println(F("[TX] Invalid packet data"));
        return RADIOLIB_ERR_PACKET_TOO_SHORT;
    }
    
    // Don't start new transmission if one is in progress
    if (isTransmitting) {
        Serial.println(F("[TX] Already transmitting, ignoring request"));
        return RADIOLIB_ERR_TX_TIMEOUT;
    }
    
    Serial.print(F("[TX] Starting transmission of "));
    Serial.print(length);
    Serial.println(F(" bytes"));
    
    // Set transmission state
    isTransmitting = true;
    
    // Set RF path to TX
    digitalWrite(RX_EN, LOW);
    digitalWrite(TX_EN, HIGH);
    
    // Start non-blocking transmission
    int state = radio.startTransmit(packet, length);
    
    if (state != RADIOLIB_ERR_NONE) {
        // If transmission failed to start, reset state
        Serial.print(F("[TX] Failed to start transmission, code "));
        Serial.println(state);
        
        isTransmitting = false;
        digitalWrite(TX_EN, LOW);
        digitalWrite(RX_EN, HIGH);
        
        // Try to resume receiving
        radio.startReceive();
    }
    
    return state;
}

// Function to handle received packets and forward to CAN
int handle_received_packet(void) {
    if (!receivedFlag) {
        return RADIOLIB_ERR_NONE; // No packet received
    }

    // Reset flag immediately
    receivedFlag = false;

    // Read received data as byte array
    uint8_t byteArr[256]; // Adjust size as needed
    int numBytes = radio.getPacketLength();

    // Validate packet length
    if (numBytes <= 0 || numBytes > sizeof(byteArr)) {
        Serial.print(F("[RX] Invalid packet length: "));
        Serial.println(numBytes);
        radio.startReceive(); // Resume listening
        return RADIOLIB_ERR_UNKNOWN;
    }

    int state = radio.readData(byteArr, numBytes);

    if (state == RADIOLIB_ERR_NONE) {
        // Packet received successfully
        Serial.println(F(""));
        Serial.println(F("=== LoRa PACKET RECEIVED ==="));
        Serial.print(F("[LoRa RX] Length: "));
        Serial.print(numBytes);
        Serial.println(F(" bytes"));

        // Print packet data in hex
        Serial.print(F("[LoRa RX] Data: "));
        for (int i = 0; i < numBytes; i++) {
            if (byteArr[i] < 0x10) Serial.print("0");
            Serial.print(byteArr[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Print signal quality
        Serial.print(F("[LoRa RX] RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        Serial.print(F("[LoRa RX] SNR: "));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        // Check if this is a CAN message structure
        if (numBytes == sizeof(CAN_message_t)) {
            CAN_message_t* canMsg = (CAN_message_t*)byteArr;

            // Validate CAN message
            if (canMsg->data_length <= 8) {
                Serial.println(F("[BRIDGE] Valid CAN message detected, forwarding to CAN bus..."));
                Serial.print(F("[BRIDGE] CAN ID: 0x"));
                Serial.print(canMsg->id, HEX);
                Serial.print(F(" | Length: "));
                Serial.println(canMsg->data_length);

                // Forward to CAN bus
                bool sent = can.write(canMsg->id, canMsg->data, canMsg->data_length);
                if (sent) {
                    Serial.println(F("[BRIDGE] Successfully forwarded to CAN bus"));
                    digitalWrite(LED_RX, HIGH);
                    delay(50);
                    digitalWrite(LED_RX, LOW);
                } else {
                    Serial.println(F("[BRIDGE] Failed to forward to CAN bus"));
                }
            } else {
                Serial.println(F("[BRIDGE] Invalid CAN data length, not forwarding"));
            }
        }

        Serial.println(F("============================"));
        Serial.println(F(""));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println(F("[LoRa RX] CRC error - corrupted packet"));
    } else {
        Serial.print(F("[LoRa RX] Failed to read packet, code "));
        Serial.println(state);
    }

    // Always resume listening after handling a packet
    int resumeState = radio.startReceive();
    if (resumeState != RADIOLIB_ERR_NONE) {
        Serial.print(F("[LoRa RX] Failed to resume listening, code "));
        Serial.println(resumeState);
    }

    return state;
}

void handle_transmission_complete(void) {
    if (!transmittedFlag) {
        return; // No transmission completed
    }
    
    // Reset flag
    transmittedFlag = false;
    isTransmitting = false;
    
    // Switch back to RX mode
    digitalWrite(TX_EN, LOW);
    digitalWrite(RX_EN, HIGH);
    
    Serial.println(F("[TX] Transmission complete - switching to RX"));
    
    // Resume receiving
    int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("[TX] Failed to resume RX after TX, code "));
        Serial.println(state);
    } else {
        Serial.println(F("[RX] Listening for packets..."));
    }
}





void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for serial connection

    // Initialize LED pins
    pinMode(LED_TX, OUTPUT);
    pinMode(LED_RX, OUTPUT);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_RX, LOW);

    // Initialize button pin
    pinMode(USER_BTN, INPUT_PULLUP);

    // Initialize RF switch pins
    pinMode(RX_EN, OUTPUT);
    pinMode(TX_EN, OUTPUT);
    digitalWrite(RX_EN, LOW);
    digitalWrite(TX_EN, LOW);

    // LED startup sequence
    Serial.println(F(""));
    Serial.println(F("=== SYSTEM STARTUP ==="));
    digitalWrite(LED_TX, HIGH);
    delay(200);
    digitalWrite(LED_RX, HIGH);
    delay(200);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_RX, LOW);
    delay(200);

    // Initialize CAN bus
    init_can();

    // Initialize radio
    init_radio();

    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════════╗"));
    Serial.println(F("║    CAN-LoRa Bidirectional Bridge Ready     ║"));
    Serial.println(F("╚════════════════════════════════════════════╝"));
    Serial.println(F(""));
    Serial.println(F("Functionality:"));
    Serial.println(F("  → CAN messages automatically forwarded to LoRa"));
    Serial.println(F("  → LoRa messages automatically forwarded to CAN"));
    Serial.println(F("  → Press USER_BTN to send test CAN message"));
    Serial.println(F(""));
    Serial.println(F("LED Indicators:"));
    Serial.println(F("  → GREEN (TX): LoRa transmission"));
    Serial.println(F("  → BLUE  (RX): CAN/LoRa reception"));
    Serial.println(F(""));
    Serial.println(F("Ready for testing..."));
    Serial.println(F(""));
}

void loop() {
    // Handle CAN messages (receive and forward over LoRa)
    handle_can_messages();

    // Handle radio operations
    handle_received_packet();
    handle_transmission_complete();

    // Handle button press for manual testing
    handle_button_press();

    // Small delay to prevent overwhelming the CPU
    delay(10);
}


/* Button handler for testing - sends CAN messages when pressed */
void handle_button_press() {
    // Read the button state
    bool reading = digitalRead(USER_BTN);

    // Check if button state changed (for debouncing)
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    // If enough time has passed since last change
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // If button state has actually changed
        if (reading != currentButtonState) {
            currentButtonState = reading;

            // Button was pressed (HIGH to LOW transition for active LOW button)
            if (currentButtonState == LOW) {
                Serial.println(F(""));
                Serial.println(F("*** BUTTON PRESSED - Sending Test CAN Message ***"));

                // Create test data with incrementing pattern + packet counter
                uint8_t testData[8];
                testData[0] = (packetCounter >> 8) & 0xFF;  // Counter high byte
                testData[1] = packetCounter & 0xFF;         // Counter low byte
                testData[2] = 0xAA;
                testData[3] = 0xBB;
                testData[4] = 0xCC;
                testData[5] = 0xDD;
                testData[6] = 0xEE;
                testData[7] = 0xFF;

                // Send on CAN bus with a unique ID
                send_can_message(0x100 + (packetCounter % 16), 8, testData);

                Serial.println(F(""));
            }
        }
    }

    lastButtonState = reading;
}

// Function to receive CAN messages and forward them over LoRa
void handle_can_messages() {
    CAN_message_t msg;

    // Check if there's a CAN message available
    if (can.available() && can.read(msg)) {
        Serial.println(F(""));
        Serial.println(F("=== CAN MESSAGE RECEIVED ==="));
        Serial.print(F("[CAN RX] ID: 0x"));
        Serial.print(msg.id, HEX);
        Serial.print(F(" | Length: "));
        Serial.println(msg.data_length);

        Serial.print(F("[CAN RX] Data: "));
        for (int i = 0; i < msg.data_length; i++) {
            if (msg.data[i] < 0x10) Serial.print("0");
            Serial.print(msg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        Serial.println(F("==========================="));
        Serial.println(F(""));

        // Forward CAN message over LoRa
        Serial.println(F("[BRIDGE] Forwarding CAN message over LoRa..."));

        // LED indicator for CAN RX
        digitalWrite(LED_RX, HIGH);

        int state = start_transmit((uint8_t*)&msg, sizeof(msg));
        if (state == RADIOLIB_ERR_NONE) {
            packetCounter++;
            digitalWrite(LED_TX, HIGH);
            delay(50);
            digitalWrite(LED_TX, LOW);
        } else {
            Serial.print(F("[BRIDGE] Failed to forward CAN message, code "));
            Serial.println(state);
        }

        digitalWrite(LED_RX, LOW);
    }
}

// Function to send CAN message directly on CAN bus
void send_can_message(uint16_t id, uint8_t data_length, uint8_t *data) {
    if (data_length > 8) {
        Serial.println(F("[CAN] Error: Data length exceeds 8 bytes"));
        return;
    }

    Serial.println(F(""));
    Serial.print(F("[CAN] Sending message | ID: 0x"));
    Serial.print(id, HEX);
    Serial.print(F(" | Length: "));
    Serial.println(data_length);

    Serial.print(F("[CAN] Data: "));
    for (int i = 0; i < data_length; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    bool sent = can.write(id, data, data_length);
    if (sent) {
        Serial.println(F("[CAN] Message sent successfully"));
    } else {
        Serial.println(F("[CAN] Failed to send message"));
    }
    Serial.println(F(""));
}

void send_test_packet() {
    // Create a test CAN message
    uint8_t testData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    Serial.println(F("[TEST] Sending test CAN message..."));
    send_can_message(0x123, 8, testData);
}
