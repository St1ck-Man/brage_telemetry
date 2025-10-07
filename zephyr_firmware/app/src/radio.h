#ifndef RADIO_H
#define RADIO_H

#include <stddef.h>
#include <stdint.h>
#include "sx128x/sx128x_hal.h"
#include "sx128x/sx128x.h"

sx128x_status_t radio_init()
sx128x_hal_status_t start_transmit(uint8_t* data, size_t len);
sx128x_hal_status_t start_receive(uint8_t timeout, sx128x_irq_mask_t irq_flags, size_t len);
sx128x_hal_status_t finish_transmit();
sx128x_hal_status_t finish_receive();


#endif // !RADIO_H
#define RADIO_H
