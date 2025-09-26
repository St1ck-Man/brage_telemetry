#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H 
#endif

/*Status LEDs*/
#define LED_TX LED_GREEN
#define LED_RX LED_BLUE

/*sx1280 control pins*/ // bottom right on the board
#define SX_DIO1 8 // PA9
#define SX_RESET 9 // PB3
#define SX_CS 10 // SPI CS / PA15
#define SX_BUSY 7 // not connected, LAMBDA80 doesn't have BUSY pin

/*RF control*/
#define TX_EN 0 // PB7
#define RX_EN 1 //
