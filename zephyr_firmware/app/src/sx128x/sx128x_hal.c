/**
 * @file      sx128x_hal.c
 *
 * @brief     Hardware Abstraction Layer implementation for SX128X in Zephyr
 * RTOS
 */

#include "sx128x_hal.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/* SPI configuration */
#define SX128X_SPI_NODE DT_NODELABEL(spi1)
#define SX128X_SPI_FREQUENCY 8000000 /* 8 MHz */

/* GPIO pins */
#define SX128X_CS_NODE                                                         \
  DT_NODELABEL(sx128x_cs) // remember to adjust in devicetree for nucleo board
#define SX128X_RESET_NODE DT_NODELABEL(sx128x_reset)
#define SX128X_BUSY_NODE DT_NODELABEL(sx128x_busy)

/* Timeout values */
#define SX128X_BUSY_TIMEOUT_MS 1000
#define SX128X_WAKEUP_DELAY_US 10

/* SPI device */
static const struct device *spi_dev = DEVICE_DT_GET(SX128X_SPI_NODE);

/* GPIO device specs */
static struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(SX128X_CS_NODE, gpios);
static struct gpio_dt_spec reset_gpio =
    GPIO_DT_SPEC_GET(SX128X_RESET_NODE, gpios);
static struct gpio_dt_spec busy_gpio =
    GPIO_DT_SPEC_GET(SX128X_BUSY_NODE, gpios);

/* SPI configuration - manual CS control */
static struct spi_config spi_cfg = {
    .frequency = SX128X_SPI_FREQUENCY,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL |
                 SPI_MODE_CPHA | SPI_OP_MODE_MASTER,
    .slave = 0,
    .cs = NULL // Manual CS control
};

/* Initialization flag */
static bool hal_initialized = false;

/**
 * @brief Wait until the radio BUSY pin goes low
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns SX128X_HAL_STATUS_OK if BUSY went low, SX128X_HAL_STATUS_ERROR on
 * timeout
 */
static sx128x_hal_status_t sx128x_hal_wait_on_busy(const void *context);

/**
 * @brief Initialize the HAL (GPIOs and SPI)
 *
 * @returns SX128X_HAL_STATUS_OK on success, SX128X_HAL_STATUS_ERROR on failure
 */
static sx128x_hal_status_t sx128x_hal_init(void);

sx128x_hal_status_t sx128x_hal_write(const void *context,
                                     const uint8_t *command,
                                     const uint16_t command_length,
                                     const uint8_t *data,
                                     const uint16_t data_length) {
  /* Initialize HAL if not already done */
  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  /* Wake up the radio if needed */
  if (sx128x_hal_wakeup(context) != SX128X_HAL_STATUS_OK) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Wait for radio to be ready */
  if (sx128x_hal_wait_on_busy(context) != SX128X_HAL_STATUS_OK) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Assert CS (active low) */
  gpio_pin_set_dt(&cs_gpio, 0);

  /* Prepare SPI buffers */
  struct spi_buf tx_bufs[2];
  struct spi_buf_set tx;

  tx_bufs[0].buf = (uint8_t *)command;
  tx_bufs[0].len = command_length;

  if (data_length > 0) {
    tx_bufs[1].buf = (uint8_t *)data;
    tx_bufs[1].len = data_length;
    tx.buffers = tx_bufs;
    tx.count = 2;
  } else {
    tx.buffers = tx_bufs;
    tx.count = 1;
  }

  /* Perform SPI write */
  int ret = spi_write(spi_dev, &spi_cfg, &tx);

  /* De-assert CS */
  gpio_pin_set_dt(&cs_gpio, 1);

  if (ret < 0) {
    return SX128X_HAL_STATUS_ERROR;
  }

  return SX128X_HAL_STATUS_OK;
}

sx128x_hal_status_t sx128x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length) {
  /* Initialize HAL if not already done */
  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  /* Wake up the radio if needed */
  if (sx128x_hal_wakeup(context) != SX128X_HAL_STATUS_OK) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Wait for radio to be ready */
  if (sx128x_hal_wait_on_busy(context) != SX128X_HAL_STATUS_OK) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Assert CS (active low) */
  gpio_pin_set_dt(&cs_gpio, 0);

  /* Prepare SPI buffers */
  struct spi_buf tx_bufs[2];
  struct spi_buf rx_bufs[2];
  struct spi_buf_set tx;
  struct spi_buf_set rx;

  /* Send command */
  tx_bufs[0].buf = (uint8_t *)command;
  tx_bufs[0].len = command_length;

  /* Send dummy bytes during data read */
  tx_bufs[1].buf = NULL;
  tx_bufs[1].len = data_length;

  tx.buffers = tx_bufs;
  tx.count = 2;

  /* Discard bytes received during command phase */
  rx_bufs[0].buf = NULL;
  rx_bufs[0].len = command_length;

  /* Receive actual data */
  rx_bufs[1].buf = data;
  rx_bufs[1].len = data_length;

  rx.buffers = rx_bufs;
  rx.count = 2;

  /* Perform SPI transceive */
  int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

  /* De-assert CS */
  gpio_pin_set_dt(&cs_gpio, 1);

  if (ret < 0) {
    return SX128X_HAL_STATUS_ERROR;
  }

  return SX128X_HAL_STATUS_OK;
}

sx128x_hal_status_t sx128x_hal_reset(const void *context) {
  /* Initialize HAL if not already done */
  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  /* Hardware reset sequence:
   * 1. Set RESET pin low (active)
   * 2. Wait for at least 100 µs
   * 3. Set RESET pin high (inactive)
   * 4. Wait for radio to boot
   */
  gpio_pin_set_dt(&reset_gpio, 0);
  k_sleep(K_USEC(100));
  gpio_pin_set_dt(&reset_gpio, 1);
  k_sleep(K_MSEC(10)); // Wait for radio to boot

  /* Wait for BUSY pin to go low */
  return sx128x_hal_wait_on_busy(context);
}

sx128x_hal_status_t sx128x_hal_wakeup(const void *context) {
  /* Initialize HAL if not already done */
  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  /* Check if radio is already awake (BUSY pin low) */
  if (gpio_pin_get_dt(&busy_gpio) == 0) {
    return SX128X_HAL_STATUS_OK;
  }

  /* Wakeup sequence for SX128X:
   * The radio wakes up by toggling NSS (CS) pin
   * 1. Assert CS low
   * 2. Wait briefly
   * 3. De-assert CS high
   * 4. Wait for BUSY to go low
   */
  gpio_pin_set_dt(&cs_gpio, 0);
  k_busy_wait(SX128X_WAKEUP_DELAY_US);
  gpio_pin_set_dt(&cs_gpio, 1);

  /* Wait for BUSY pin to go low */
  return sx128x_hal_wait_on_busy(context);
}

static sx128x_hal_status_t sx128x_hal_wait_on_busy(const void *context) {
  uint32_t timeout = SX128X_BUSY_TIMEOUT_MS;

  /* Wait while BUSY pin is high */
  while (gpio_pin_get_dt(&busy_gpio) != 0) {
    k_sleep(K_USEC(100));
    timeout--;

    if (timeout == 0) {
      /* Timeout occurred */
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  return SX128X_HAL_STATUS_OK;
}

static sx128x_hal_status_t sx128x_hal_init(void) {
  /* Check if SPI device is ready */
  if (!device_is_ready(spi_dev)) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Configure CS GPIO as output, initially high (inactive) */
  if (!device_is_ready(cs_gpio.port)) {
    return SX128X_HAL_STATUS_ERROR;
  }
  if (gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Configure RESET GPIO as output, initially high (inactive) */
  if (!device_is_ready(reset_gpio.port)) {
    return SX128X_HAL_STATUS_ERROR;
  }
  if (gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_ACTIVE) < 0) {
    return SX128X_HAL_STATUS_ERROR;
  }

  /* Configure BUSY GPIO as input */
  if (!device_is_ready(busy_gpio.port)) {
    return SX128X_HAL_STATUS_ERROR;
  }
  if (gpio_pin_configure_dt(&busy_gpio, GPIO_INPUT) < 0) {
    return SX128X_HAL_STATUS_ERROR;
  }

  hal_initialized = true;
  return SX128X_HAL_STATUS_OK;
}
