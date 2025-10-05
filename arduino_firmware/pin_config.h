#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/*Status LEDs*/
#define LED_TX LED_GREEN // Use built-in LED or define specific pin like PA5
#define LED_RX LED_BLUE

/*User Button for testing*/
// #define USER_BTN PC13 // Built-in button on Nucleo boards

/*sx1280 control pins*/ // bottom right on the board
#define SX_DIO1 8       // PA9
#define SX_RESET 9      // PB3
#define SX_CS 10        // SPI CS / PA15
#define SX_BUSY 7       // not connected, LAMBDA80 doesn't have BUSY pin

/*RF control*/
#define TX_EN 0 // PB7
#define RX_EN 1 //

/*CAN pins - using FDCAN on STM32*/
#define CAN_RX PA11
#define CAN_TX PA12

// /* Status LEDs */
// #define LED_TX PB12
// #define LED_RX PB13
//
// /* sx1280 control pins */
// #define SX_CS PB0
// #define SX_DIO1 PB3
// #define SX_RESET PB6
// #define SX_BUSY PA15
//
// /* SPI (includes SX_CS)*/
// #define SCK PA5
// #define MISO PA6
// #define MOSI PA7
//
// /* PA control */
// #define PA_EN PC13
// #define PA_TX_EN PC14
// #define PA_RX_EN PC15

#endif
