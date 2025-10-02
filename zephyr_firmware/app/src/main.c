/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample CAN implementation from zephyr */
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

/* Thread configuration constants */
#define RX_THREAD_STACK_SIZE 512         // Stack size for receive thread
#define RX_THREAD_PRIORITY 2             // Priority level for RX thread
#define STATE_POLL_THREAD_STACK_SIZE 512 // Stack size for state polling thread
#define STATE_POLL_THREAD_PRIORITY 2     // Priority level for state thread

/* CAN message identifiers */
#define LED_MSG_ID 0x10        // Standard CAN ID for LED control messages
#define COUNTER_MSG_ID 0x12345 // Extended CAN ID for counter messages

/* LED control values */
#define SET_LED 1   // Value to turn LED on
#define RESET_LED 0 // Value to turn LED off

/* Timing constant */
#define SLEEP_TIME K_MSEC(250) // 250ms delay between transmissions

/* Allocate memory for thread stacks */
K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);

/* Get the CAN device from device tree */
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/* Get LED GPIO specification from device tree (or use empty spec if not
 * available) */
struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

/* Thread control blocks */
struct k_thread rx_thread_data;
struct k_thread poll_state_thread_data;

/* Work items for deferred processing */
struct k_work_poll change_led_work; // Polling work for LED changes
struct k_work state_change_work;    // Work item for state change callbacks

/* Global state variables */
enum can_state current_state;           // Current CAN bus state
struct can_bus_err_cnt current_err_cnt; // Current error counters

/* Message queues for CAN frames - macro creates queue with 2 frame capacity */
CAN_MSGQ_DEFINE(change_led_msgq, 2); // Queue for LED control messages
CAN_MSGQ_DEFINE(counter_msgq, 2);    // Queue for counter messages

/* Poll event for LED message queue - triggers when data is available */
static struct k_poll_event change_led_events[1] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY, &change_led_msgq,
                                    0)};

/**
 * Callback function called when a CAN frame transmission completes
 * @param dev: The CAN device
 * @param error: Error code (0 = success)
 * @param arg: User-provided argument (sender identification string)
 */
void tx_irq_callback(const struct device *dev, int error, void *arg) {
  char *sender = (char *)arg;

  ARG_UNUSED(dev);

  // Only print if there was an error
  if (error != 0) {
    printf("Callback! error-code: %d\nSender: %s\n", error, sender);
  }
}

/**
 * Thread function that receives counter messages from CAN bus
 * Filters for extended ID messages with COUNTER_MSG_ID
 */
void rx_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  /* Configure filter for extended CAN frames with specific ID */
  const struct can_filter filter = {
      .flags = CAN_FILTER_IDE, // Filter for extended ID frames
      .id = COUNTER_MSG_ID,    // Match this specific ID
      .mask = CAN_EXT_ID_MASK  // Check all bits of extended ID
  };
  struct can_frame frame;
  int filter_id;

  /* Add filter and attach to message queue - received frames go directly to
   * queue */
  filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filter);
  printf("Counter filter id: %d\n", filter_id);

  /* Main receive loop - runs forever */
  while (1) {
    /* Block until a frame is available in the queue */
    k_msgq_get(&counter_msgq, &frame, K_FOREVER);

    /* Skip Remote Transmission Request frames if RTR is enabled */
    if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) &&
        (frame.flags & CAN_FRAME_RTR) != 0U) {
      continue;
    }

    /* Validate data length - expecting 2 bytes for uint16_t counter */
    if (frame.dlc != 2U) {
      printf("Wrong data length: %u\n", frame.dlc);
      continue;
    }

    /* Extract counter value - convert from big-endian network byte order */
    printf("Counter received: %u\n",
           sys_be16_to_cpu(UNALIGNED_GET((uint16_t *)&frame.data)));
  }
}

/**
 * Work handler that processes LED control messages
 * Executes in system workqueue context when polled event triggers
 */
void change_led_work_handler(struct k_work *work) {
  struct can_frame frame;
  int ret;

  /* Process all available messages in the queue */
  while (k_msgq_get(&change_led_msgq, &frame, K_NO_WAIT) == 0) {
    /* Skip RTR frames if enabled */
    if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) &&
        (frame.flags & CAN_FRAME_RTR) != 0U) {
      continue;
    }

    /* Control LED based on frame data */
    if (led.port == NULL) {
      /* No LED available - just print status */
      printf("LED %s\n", frame.data[0] == SET_LED ? "ON" : "OFF");
    } else {
      /* Set GPIO pin high (1) or low (0) */
      gpio_pin_set(led.port, led.pin, frame.data[0] == SET_LED ? 1 : 0);
    }
  }

  /* Resubmit the polling work to wait for next message */
  ret = k_work_poll_submit(&change_led_work, change_led_events,
                           ARRAY_SIZE(change_led_events), K_FOREVER);
  if (ret != 0) {
    printf("Failed to resubmit msgq polling: %d", ret);
  }
}

/**
 * Convert CAN state enum to human-readable string
 * @param state: The CAN controller state
 * @return: String representation of the state
 */
char *state_to_str(enum can_state state) {
  switch (state) {
  case CAN_STATE_ERROR_ACTIVE:
    return "error-active"; // Normal operation
  case CAN_STATE_ERROR_WARNING:
    return "error-warning"; // Error count exceeded warning threshold
  case CAN_STATE_ERROR_PASSIVE:
    return "error-passive"; // Error count exceeded passive threshold
  case CAN_STATE_BUS_OFF:
    return "bus-off"; // Too many errors, disconnected from bus
  case CAN_STATE_STOPPED:
    return "stopped"; // Controller is stopped
  default:
    return "unknown";
  }
}

/**
 * Thread that periodically polls CAN controller state and error counts
 * Prints updates when state or error counts change
 */
void poll_state_thread(void *unused1, void *unused2, void *unused3) {
  struct can_bus_err_cnt err_cnt = {0, 0};
  struct can_bus_err_cnt err_cnt_prev = {0, 0};
  enum can_state state_prev = CAN_STATE_ERROR_ACTIVE;
  enum can_state state;
  int err;

  while (1) {
    /* Query current state and error counters */
    err = can_get_state(can_dev, &state, &err_cnt);
    if (err != 0) {
      printf("Failed to get CAN controller state: %d", err);
      k_sleep(K_MSEC(100));
      continue;
    }

    /* Check if anything changed since last poll */
    if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
        err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt || state_prev != state) {

      /* Update previous values */
      err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
      err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
      state_prev = state;

      /* Print current status */
      printf("state: %s\n"
             "rx error count: %d\n"
             "tx error count: %d\n",
             state_to_str(state), err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);
    } else {
      /* No change - sleep before next poll */
      k_sleep(K_MSEC(100));
    }
  }
}

/**
 * Work handler for state change callbacks
 * Prints state information when CAN controller state changes
 */
void state_change_work_handler(struct k_work *work) {
  printf("State Change ISR\nstate: %s\n"
         "rx error count: %d\n"
         "tx error count: %d\n",
         state_to_str(current_state), current_err_cnt.rx_err_cnt,
         current_err_cnt.tx_err_cnt);
}

/**
 * Interrupt callback for CAN state changes
 * Executes in ISR context - defers work to system workqueue
 * @param dev: CAN device
 * @param state: New state
 * @param err_cnt: Current error counters
 * @param user_data: User-provided work item pointer
 */
void state_change_callback(const struct device *dev, enum can_state state,
                           struct can_bus_err_cnt err_cnt, void *user_data) {
  struct k_work *work = (struct k_work *)user_data;

  ARG_UNUSED(dev);

  /* Store state in global variables */
  current_state = state;
  current_err_cnt = err_cnt;

  /* Submit work item for processing outside ISR context */
  k_work_submit(work);
}

/**
 * Main function - initializes CAN bus and runs transmission loop
 */
int main(void) {
  /* Filter for standard ID LED control messages */
  const struct can_filter change_led_filter = {
      .flags = 0U,            // Standard ID (not extended)
      .id = LED_MSG_ID,       // Match LED message ID
      .mask = CAN_STD_ID_MASK // Check all standard ID bits
  };

  /* Frame for LED control - standard ID, 1 byte of data */
  struct can_frame change_led_frame = {
      .flags = 0,
      .id = LED_MSG_ID,
      .dlc = 1 // Data length: 1 byte
  };

  /* Frame for counter - extended ID, 2 bytes of data */
  struct can_frame counter_frame = {
      .flags = CAN_FRAME_IDE, // Extended ID flag
      .id = COUNTER_MSG_ID,
      .dlc = 2 // Data length: 2 bytes
  };

  uint8_t toggle = 1;   // Toggle variable for LED state
  uint16_t counter = 0; // Counter value to transmit
  k_tid_t rx_tid, get_state_tid;
  int ret;

  /* Verify CAN device is ready */
  if (!device_is_ready(can_dev)) {
    printf("CAN: Device %s not ready.\n", can_dev->name);
    return 0;
  }

#ifdef CONFIG_LOOPBACK_MODE
  /* Enable loopback mode for testing without physical bus */
  ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
  if (ret != 0) {
    printf("Error setting CAN mode [%d]", ret);
    return 0;
  }
#endif

  /* Start the CAN controller */
  ret = can_start(can_dev);
  if (ret != 0) {
    printf("Error starting CAN controller [%d]", ret);
    return 0;
  }

  /* Initialize LED GPIO if available */
  if (led.port != NULL) {
    if (!gpio_is_ready_dt(&led)) {
      printf("LED: Device %s not ready.\n", led.port->name);
      return 0;
    }
    /* Configure LED pin as output, initially high */
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_HIGH);
    if (ret < 0) {
      printf("Error setting LED pin to output mode [%d]", ret);
      led.port = NULL; // Disable LED on error
    }
  }

  /* Initialize work items */
  k_work_init(&state_change_work, state_change_work_handler);
  k_work_poll_init(&change_led_work, change_led_work_handler);

  /* Add receive filter for LED messages */
  ret = can_add_rx_filter_msgq(can_dev, &change_led_msgq, &change_led_filter);
  if (ret == -ENOSPC) {
    printf("Error, no filter available!\n");
    return 0;
  }

  printf("Change LED filter ID: %d\n", ret);

  /* Submit polling work for LED messages */
  ret = k_work_poll_submit(&change_led_work, change_led_events,
                           ARRAY_SIZE(change_led_events), K_FOREVER);
  if (ret != 0) {
    printf("Failed to submit msgq polling: %d", ret);
    return 0;
  }

  /* Create thread to receive counter messages */
  rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
                           K_THREAD_STACK_SIZEOF(rx_thread_stack), rx_thread,
                           NULL, NULL, NULL, RX_THREAD_PRIORITY, 0, K_NO_WAIT);
  if (!rx_tid) {
    printf("ERROR spawning rx thread\n");
  }

  /* Create thread to poll CAN controller state */
  get_state_tid = k_thread_create(&poll_state_thread_data, poll_state_stack,
                                  K_THREAD_STACK_SIZEOF(poll_state_stack),
                                  poll_state_thread, NULL, NULL, NULL,
                                  STATE_POLL_THREAD_PRIORITY, 0, K_NO_WAIT);
  if (!get_state_tid) {
    printf("ERROR spawning poll_state_thread\n");
  }

  /* Register callback for CAN state changes */
  can_set_state_change_callback(can_dev, state_change_callback,
                                &state_change_work);

  printf("Finished init.\n");

  /* Main transmission loop */
  while (1) {
    /* Toggle LED state (alternates between on/off) */
    change_led_frame.data[0] = toggle++ & 0x01 ? SET_LED : RESET_LED;

    /* Send LED control frame - non-blocking with callback */
    can_send(can_dev, &change_led_frame, K_FOREVER, tx_irq_callback,
             "LED change");
    k_sleep(SLEEP_TIME);

    /* Prepare counter frame - store as big-endian uint16_t */
    UNALIGNED_PUT(sys_cpu_to_be16(counter), (uint16_t *)&counter_frame.data[0]);
    counter++;

    /* Send counter frame - blocking call with 100ms timeout */
    can_send(can_dev, &counter_frame, K_MSEC(100), NULL, NULL);
    k_sleep(SLEEP_TIME);
  }
}
