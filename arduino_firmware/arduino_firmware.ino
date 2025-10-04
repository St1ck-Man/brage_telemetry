#include "pin_config.h"
#include <RadioLib.h>

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

// Function to handle received packets
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
        Serial.println(F("=== PACKET RECEIVED ==="));
        Serial.print(F("[RX] Length: "));
        Serial.print(numBytes);
        Serial.println(F(" bytes"));
        
        // Print packet data in hex
        Serial.print(F("[RX] Data: "));
        for (int i = 0; i < numBytes; i++) {
            if (byteArr[i] < 0x10) Serial.print("0");
            Serial.print(byteArr[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Print signal quality
        Serial.print(F("[RX] RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
        
        Serial.print(F("[RX] SNR: "));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));
        
        Serial.println(F("====================="));
        Serial.println(F(""));
        
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println(F("[RX] CRC error - corrupted packet"));
    } else {
        Serial.print(F("[RX] Failed to read packet, code "));
        Serial.println(state);
    }
    
    // Always resume listening after handling a packet
    int resumeState = radio.startReceive();
    if (resumeState != RADIOLIB_ERR_NONE) {
        Serial.print(F("[RX] Failed to resume listening, code "));
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
    
    // Initialize button pin
    // pinMode(USER_BTN, INPUT_PULLUP);
    
    // Initialize RF switch pins (add these pin definitions to your constants)
    pinMode(PA_RX_EN, OUTPUT);
    pinMode(PA_TX_EN, OUTPUT);
    pinMode(PA_EN, OUTPUT);
    digitalWrite(RX_EN, LOW);
    digitalWrite(TX_EN, LOW);
    
    // Initialize radio
    init_radio();
    
    Serial.println(F("=== Radio System Ready ==="));
    // Serial.println(F("Press USER_BTN to send a packet"));
    Serial.println(F("Listening for incoming packets..."));
    Serial.println(F(""));
}

void loop() {
    // Handle radio operations
    handle_received_packet();
    handle_transmission_complete();
    
    send_test_packet();
    // Handle user input
    // handle_button_press();
    
    // Add small delay to prevent overwhelming the CPU
    delay(5000); // wait for 5 seconds
}


/* Just for testing with nucleo */
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
                send_test_packet();
            }
        }
    }
    
    lastButtonState = reading;
}

typedef struct {
  uint16_t id; // 11-bit identifier (0-0x7FF)
  uint8_t data_length; // length field 
  uint8_t data[8]; // 8 byte data payload
} CAN_message;

void send_test_packet(uint16_t id, uint8_t data_length, uint8_t *data) {
    // Create a test CAN message to be sent over LoRa 
    CAN_message msg;

    msg.id = id;
    msg.data_length = data_length;
    memset(msg.data, 0, 8); // initialize all bytes to 0 
    if (data != NULL) {
      memcpy(msg.data, data, msg.data_length);
    }
    
    Serial.println("Sending data: ");    
    Serial.print("\tID: ");
    Serial.println(msg->id);
    
    int state = start_transmit(packet, sizeof(packet));
    if (state == RADIOLIB_ERR_NONE) {
        packetCounter++; // Only increment if transmission started successfully
    } else {
        Serial.print(F("[USER] Failed to start transmission, code "));
        Serial.println(state);
    }
}
