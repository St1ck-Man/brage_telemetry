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
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sx128x_hal, CONFIG_SX128X_LOG_LEVEL);

#define SX128X_SPI_NODE DT_NODELABEL(spi1)
#define SX128X_DEVICE_NODE DT_NODELABEL(sx128x)
#define SX128X_SPI_FREQ 8000000

#define SX128X_BUSY_TIMEOUT_MS 100
#define SX128X_WAKEUP_DELAY_US 10

static const struct device *spi_dev = DEVICE_DT_GET(SX128X_SPI_NODE);

static struct gpio_dt_spec reset_gpio =
    GPIO_DT_SPEC_GET(SX128X_DEVICE_NODE, reset_gpios);
static struct gpio_dt_spec busy_gpio =
    GPIO_DT_SPEC_GET(SX128X_DEVICE_NODE, busy_gpios);

// SPI configuration
static struct spi_config spi_cfg = {
    .frequency = SX128X_SPI_FREQ,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
    .cs = {
        .gpio = GPIO_DT_SPEC_GET(SX128X_DEVICE_NODE, cs_gpios),
        .delay = 0,
    }};

/* Initialization flag */
static bool hal_initialized = false;

/**
 * Waits until the radio BUSY pin goes low
 */
static sx128x_hal_status_t sx128x_hal_wait_on_busy(const void *context);

/**
 * Initialize HAL (GPIOs and SPI)
 * Returns sx128x_hal_status_t
 */
static sx128x_hal_status_t sx128x_hal_init(void);

/**
 * Radio data transfer - write
 *
 * context: Pointer to the HAL context
 * command: Pointer to the command buffer
 * command_length: Length of the command buffer
 * data: Pointer to the data buffer
 * data_length: Length of the data buffer
 *
 * returns: sx128x_hal_status_t (OK or ERROR)
 */
sx128x_hal_status_t sx128x_hal_write(const void *context,
                                     const uint8_t *command,
                                     const uint16_t command_length,
                                     const uint8_t *data,
                                     const uint16_t data_length) {

  LOG_DBG("Write: cmd_len=%u data_len=%u", command_length, data_length);

  // initialize hal if not already done
  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      LOG_ERR("HAL initialization failed");
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  // Wake up the radio if needed
  if (sx128x_hal_wakeup(context) != SX128X_HAL_STATUS_OK) {
    LOG_ERR("Wakeup failed");
    return SX128X_HAL_STATUS_ERROR;
  }

  if (sx128x_hal_wait_on_busy(context) != SX128X_HAL_STATUS_OK) {
    LOG_ERR("Wait on busy timeout before write");
    return SX128X_HAL_STATUS_ERROR;
  }

  gpio_pin_set_dt(&spi_cfg.cs.gpio, 0);

  struct spi_buf tx_bufs[2];
  struct spi_buf_set tx;

  // set first buffer to command
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

  // start writing to device
  int ret = spi_write(spi_dev, &spi_cfg, &tx);

  gpio_pin_set_dt(&spi_cfg.cs.gpio, 1); // deselect device

  if (ret < 0) {
    LOG_ERR("SPI write failed: %d", ret);
    return SX128X_HAL_STATUS_ERROR;
  }

  LOG_DBG("Write completed successfully");
  return SX128X_HAL_STATUS_OK;
}

/*
 * Read received data
 *
 * context: pointer to HAL context
 * command: pointer to command buffer
 * command_length: length of command
 * data: pointer to data buffer for received data
 * data_length: size of buffer to be received
 *
 * returns: sx128x_hal_status_t (OK or ERROR)
 */
sx128x_hal_status_t sx128x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length) {

  LOG_DBG("Read: cmd_len=%u data_len=%u", command_length, data_length);

  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      LOG_ERR("HAL initialization failed");
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  if (sx128x_hal_wakeup(context) != SX128X_HAL_STATUS_OK) {
    LOG_ERR("Wakeup failed");
    return SX128X_HAL_STATUS_ERROR;
  }

  if (sx128x_hal_wait_on_busy(context) != SX128X_HAL_STATUS_OK) {
    LOG_ERR("Wait on busy timeout before read");
    return SX128X_HAL_STATUS_ERROR;
  }

  gpio_pin_set_dt(&spi_cfg.cs.gpio, 0);

  struct spi_buf tx_bufs[2]; // command buffers
  struct spi_buf rx_bufs[2]; // received data buffers
  struct spi_buf_set tx;
  struct spi_buf_set rx;

  tx_bufs[0].buf = (uint8_t *)command;
  tx_bufs[0].len = command_length;

  // Send dummy bytes during data read
  tx_bufs[1].buf = NULL;
  tx_bufs[1].len = data_length;

  tx.buffers = tx_bufs;
  tx.count = 2;

  // Discard bytes received during command phase
  rx_bufs[0].buf = NULL;
  rx_bufs[0].len = command_length;

  // Receive actual data
  rx_bufs[1].buf = data;
  rx_bufs[1].len = data_length;

  rx.buffers = rx_bufs;
  rx.count = 2;

  int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

  gpio_pin_set_dt(&spi_cfg.cs.gpio, 1); // deselect device

  if (ret < 0) {
    LOG_ERR("SPI read failed: %d", ret);
    return SX128X_HAL_STATUS_ERROR;
  }

  LOG_DBG("Read completed successfully");
  return SX128X_HAL_STATUS_OK;
}

/*
 * Reset radio
 *
 * context: pointer to HAL context
 *
 * returns: sx128x_hal_status_t (OK or ERROR)
 *
 */
sx128x_hal_status_t sx128x_hal_reset(const void *context) {

  LOG_INF("Resetting SX128x radio");

  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      LOG_ERR("HAL initialization failed");
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

  sx128x_hal_status_t status = sx128x_hal_wait_on_busy(context);
  
  if (status == SX128X_HAL_STATUS_OK) {
    LOG_INF("Radio reset completed successfully");
  } else {
    LOG_ERR("Radio reset failed - busy timeout");
  }

  return status;
}

sx128x_hal_status_t sx128x_hal_wakeup(const void *context) {

  if (!hal_initialized) {
    if (sx128x_hal_init() != SX128X_HAL_STATUS_OK) {
      LOG_ERR("HAL initialization failed");
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  if (gpio_pin_get_dt(&busy_gpio) == 0) {
    LOG_DBG("Radio already awake");
    return SX128X_HAL_STATUS_OK;
  }

  LOG_DBG("Waking up radio");

  /* Wakeup sequence for SX128X:
   * The radio wakes up by toggling NSS (CS) pin
   * 1. Assert CS
   * 2. Wait briefly
   * 3. De-assert CS
   * 4. Wait for BUSY to go low
   */
  gpio_pin_set_dt(&spi_cfg.cs.gpio, 0);
  k_busy_wait(SX128X_WAKEUP_DELAY_US);
  gpio_pin_set_dt(&spi_cfg.cs.gpio, 1);

  return sx128x_hal_wait_on_busy(context);
}

static sx128x_hal_status_t sx128x_hal_wait_on_busy(const void *context) {
  // Fast path: busy wait for first 100µs (most operations complete here)
  for (int i = 0; i < 10; i++) {
    if (gpio_pin_get_dt(&busy_gpio) == 0) {
      return SX128X_HAL_STATUS_OK;
    }
    k_busy_wait(10);  // 10µs × 10 = 100µs total
  }

  // Slow path: yield CPU for longer operations
  uint32_t timeout_ms = SX128X_BUSY_TIMEOUT_MS;
  
  LOG_DBG("Waiting for BUSY pin (slow path)");

  while (gpio_pin_get_dt(&busy_gpio) != 0) {
    k_sleep(K_MSEC(1));
    timeout_ms--;

    if (timeout_ms == 0) {
      LOG_ERR("BUSY pin timeout after %d ms", SX128X_BUSY_TIMEOUT_MS);
      return SX128X_HAL_STATUS_ERROR;
    }
  }

  LOG_DBG("BUSY pin cleared");
  return SX128X_HAL_STATUS_OK;
}

static sx128x_hal_status_t sx128x_hal_init(void) {

  LOG_INF("Initializing SX128x HAL");

  if (!device_is_ready(spi_dev)) {
    LOG_ERR("SPI device not ready");
    return SX128X_HAL_STATUS_ERROR;
  }
  LOG_DBG("SPI device ready");

  if (!device_is_ready(spi_cfg.cs.gpio.port)) {
    LOG_ERR("CS GPIO port not ready");
    return SX128X_HAL_STATUS_ERROR;
  }
  if (gpio_pin_configure_dt(&spi_cfg.cs.gpio, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("CS GPIO configuration failed");
    return SX128X_HAL_STATUS_ERROR;
  }
  LOG_DBG("CS GPIO configured");

  if (!device_is_ready(reset_gpio.port)) {
    LOG_ERR("RESET GPIO port not ready");
    return SX128X_HAL_STATUS_ERROR;
  }
  if (gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_ACTIVE) < 0) {
    LOG_ERR("RESET GPIO configuration failed");
    return SX128X_HAL_STATUS_ERROR;
  }
  LOG_DBG("RESET GPIO configured");

  if (!device_is_ready(busy_gpio.port)) {
    LOG_ERR("BUSY GPIO port not ready");
    return SX128X_HAL_STATUS_ERROR;
  }
  if (gpio_pin_configure_dt(&busy_gpio, GPIO_INPUT) < 0) {
    LOG_ERR("BUSY GPIO configuration failed");
    return SX128X_HAL_STATUS_ERROR;
  }
  LOG_DBG("BUSY GPIO configured");

  hal_initialized = true;
  LOG_INF("SX128x HAL initialized successfully");
  return SX128X_HAL_STATUS_OK;
}
