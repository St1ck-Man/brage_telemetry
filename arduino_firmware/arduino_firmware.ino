#include <RadioLib.h>

#define LED_TX LED_GREEN
#define LED_RX LED_BLUE

/*sx1280 control pins*/ // bottom right on the board
#define SX_DIO1 8 // PA9
#define SX_RESET 9 // PB3
#define SX_CS 10 // SPI CS / PA15
#define SX_BUSY 7 // not connected, LAMBDA80 doesn't have BUSY pin

/*RF control*/
#define TX_EN 0 // PB7

SX1280 radio = new Module(SX_CS, SX_DIO1, SX_RESET, SX_BUSY);

int transmissionState = RADIOLIB_ERR_NONE;

volatile bool transmittedFlag = false;

void setFlag(void) {
  transmittedFlag = true;
}

void setup() {
  Serial.begin(9600);

  // initialize SX1280 with default settings
  Serial.print(F("[SX1280] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  pinMode(TX_EN, OUTPUT);
  digitalWrite(TX_EN, HIGH);

  // set the function that will be called
  // when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  // start transmitting the first packet
  Serial.print(F("[SX1280] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  transmissionState = radio.startTransmit("Hello World!");

  // you can also transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    state = radio.startTransmit(byteArr, 8);
  */
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
  // check if the previous transmission finished
  if(transmittedFlag) {
    // reset flag
    transmittedFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio.finishTransmit();

    // wait a second before transmitting again
    delay(1000);

    // send another one
    Serial.print(F("[SX1280] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    String str = "Hello World! #" + String(count++);
    transmissionState = radio.startTransmit(str);

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      transmissionState = radio.startTransmit(byteArr, 8);
    */
  }
}