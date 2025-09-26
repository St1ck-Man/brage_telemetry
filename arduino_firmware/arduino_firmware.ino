#include "pin_config.h"
#include <RadioLib.h>

// Global state variables
volatile bool isTransmitting = false;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50ms debounce
uint32_t packetCounter = 0;  // For unique packet identification

void setup() {
    Serial.begin(115200);
    
    // Initialize button pin
    pinMode(USER_BTN, INPUT_PULLUP);
    
    // Initialize RF switch pins (add these pin definitions to your constants)
    pinMode(RX_EN, OUTPUT);
    pinMode(TX_EN, OUTPUT);
    digitalWrite(RX_EN, LOW);
    digitalWrite(TX_EN, LOW);
    
    // Initialize radio
    init_radio();
    
    Serial.println(F("=== Radio System Ready ==="));
    Serial.println(F("Press USER_BTN to send a packet"));
    Serial.println(F("Listening for incoming packets..."));
    Serial.println(F(""));
}

void loop() {
    // Handle radio operations
    handle_received_packet();
    handle_transmission_complete();
    
    // Handle user input
    handle_button_press();
    
    // Add small delay to prevent overwhelming the CPU
    delay(1);
}


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

void send_test_packet() {
    // Create a test packet with counter and some data
    uint8_t packet[8];
    
    // Add packet counter (4 bytes)
    packet[0] = (packetCounter >> 24) & 0xFF;
    packet[1] = (packetCounter >> 16) & 0xFF;
    packet[2] = (packetCounter >> 8) & 0xFF;
    packet[3] = packetCounter & 0xFF;
    
    // Add some test data
    packet[4] = 0xAA;
    packet[5] = 0xBB;
    packet[6] = 0xCC;
    packet[7] = 0xDD;
    
    Serial.println(F(""));
    Serial.print(F("[USER] Button pressed - sending packet #"));
    Serial.println(packetCounter);
    
    int state = start_transmit(packet, sizeof(packet));
    if (state == RADIOLIB_ERR_NONE) {
        packetCounter++; // Only increment if transmission started successfully
    } else {
        Serial.print(F("[USER] Failed to start transmission, code "));
        Serial.println(state);
    }
}
