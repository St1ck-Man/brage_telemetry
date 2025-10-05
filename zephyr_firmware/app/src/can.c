#include "can.h"
#include <string.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_module, LOG_LEVEL_DBG);

/* CAN device */
static const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

/* Message queues */
K_MSGQ_DEFINE(can_rx_queue, sizeof(CanMsg), 32, 4);
K_MSGQ_DEFINE(can_tx_queue, sizeof(CanMsg), 32, 4);

/* Thread stacks */
#define CAN_RX_THREAD_STACK_SIZE 1024
#define CAN_TX_THREAD_STACK_SIZE 1024
#define CAN_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(can_rx_thread_stack, CAN_RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(can_tx_thread_stack, CAN_TX_THREAD_STACK_SIZE);

static struct k_thread can_rx_thread_data;
static struct k_thread can_tx_thread_data;

/* CAN RX callback */
static void can_rx_callback(const struct device *dev, struct can_frame *frame,
                            void *user_data) {
  ARG_UNUSED(dev);
  ARG_UNUSED(user_data);

  /* Convert to CanMsg */
  CanMsg msg;
  msg.id = frame->id;
  msg.length = frame->dlc;

  if (msg.length > 64) {
    LOG_WRN("Received CAN frame with length > 64, truncating");
    msg.length = 64;
  }

  memcpy(msg.data, frame->data, msg.length);

  /* Queue the message for radio transmission */
  int ret = k_msgq_put(&can_tx_queue, &msg, K_NO_WAIT);
  if (ret != 0) {
    LOG_WRN("CAN TX queue full, dropping message ID 0x%03x", msg.id);
  }
}

/* CAN RX thread - sets up filter and waits */
static void can_rx_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_INF("CAN RX thread started");

  /* Setup RX filter for standard IDs 0-255 */
  struct can_filter filter = {
      .flags = CAN_FILTER_IDE,
      .id = 0x000,
      .mask = CAN_STD_ID_MASK & ~0xFF /* Accept IDs 0x000-0x0FF */
  };

  int filter_id = can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);
  if (filter_id < 0) {
    LOG_ERR("Failed to add CAN RX filter: %d", filter_id);
    return;
  }

  LOG_INF("CAN RX filter installed (ID: %d), accepting IDs 0-255", filter_id);

  /* Thread keeps running - callbacks handle RX */
  while (1) {
    k_sleep(K_FOREVER);
  }
}

/* CAN TX thread - sends messages from rx_queue to CAN bus */
static void can_tx_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_INF("CAN TX thread started");

  CanMsg msg;

  while (1) {
    /* Block waiting for message from radio */
    int ret = k_msgq_get(&can_rx_queue, &msg, K_FOREVER);
    if (ret != 0) {
      LOG_ERR("Failed to get message from RX queue: %d", ret);
      continue;
    }

    /* Validate message */
    if (msg.id > 0xFF) {
      LOG_WRN("CAN message ID 0x%03x out of range (0-255), dropping", msg.id);
      continue;
    }

    if (msg.length > 64) {
      LOG_WRN("CAN message length %d > 64, truncating", msg.length);
      msg.length = 64;
    }

    /* Convert to Zephyr CAN frame */
    struct can_frame frame = {
        .flags =
            CAN_FRAME_FDF | CAN_FRAME_BRS, /* FD mode with bit rate switch */
        .id = msg.id,
        .dlc = msg.length};

    memcpy(frame.data, msg.data, msg.length);

    /* Send the frame */
    ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {
      LOG_ERR("Failed to send CAN frame ID 0x%03x: %d", msg.id, ret);
    } else {
      LOG_DBG("Sent CAN frame ID 0x%03x, len %d", msg.id, msg.length);
    }
  }
}

int can_init(void) {
  if (!device_is_ready(can_dev)) {
    LOG_ERR("CAN device not ready");
    return -ENODEV;
  }

  LOG_INF("CAN device ready");

  /* Configure CAN timing for CAN-FD */
  /* Nominal bitrate: 500 kbps, Data bitrate: 1 Mbps */
  struct can_timing timing = {.sjw = 1,
                              .prop_seg = 0,
                              .phase_seg1 = 13,
                              .phase_seg2 = 2,
                              .prescaler = 6};

  struct can_timing timing_data = {.sjw = 1,
                                   .prop_seg = 0,
                                   .phase_seg1 = 6,
                                   .phase_seg2 = 1,
                                   .prescaler = 6};

  int ret = can_set_timing(can_dev, &timing);
  if (ret != 0) {
    LOG_ERR("Failed to set CAN timing: %d", ret);
    return ret;
  }

  ret = can_set_timing_data(can_dev, &timing_data);
  if (ret != 0) {
    LOG_ERR("Failed to set CAN data timing: %d", ret);
    return ret;
  }

  /* Set CAN mode to FD */
  ret = can_set_mode(can_dev, CAN_MODE_FD);
  if (ret != 0) {
    LOG_ERR("Failed to set CAN FD mode: %d", ret);
    return ret;
  }

  /* Start CAN controller */
  ret = can_start(can_dev);
  if (ret != 0) {
    LOG_ERR("Failed to start CAN: %d", ret);
    return ret;
  }

  LOG_INF("CAN controller started in FD mode (500k/1M)");

  /* Create RX thread */
  k_thread_create(&can_rx_thread_data, can_rx_thread_stack,
                  K_THREAD_STACK_SIZEOF(can_rx_thread_stack), can_rx_thread,
                  NULL, NULL, NULL, CAN_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&can_rx_thread_data, "can_rx");

  /* Create TX thread */
  k_thread_create(&can_tx_thread_data, can_tx_thread_stack,
                  K_THREAD_STACK_SIZEOF(can_tx_thread_stack), can_tx_thread,
                  NULL, NULL, NULL, CAN_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&can_tx_thread_data, "can_tx");

  LOG_INF("CAN threads started");

  return 0;
}

int can_queue_rx_msg(const CanMsg *msg) {
  if (!msg) {
    return -EINVAL;
  }

  /* Queue message for transmission on CAN bus */
  int ret = k_msgq_put(&can_rx_queue, msg, K_NO_WAIT);
  if (ret != 0) {
    return -ENOMEM;
  }

  return 0;
}

int can_dequeue_tx_msg(CanMsg *msg, k_timeout_t timeout) {
  if (!msg) {
    return -EINVAL;
  }

  /* Dequeue message received from CAN bus */
  int ret = k_msgq_get(&can_tx_queue, msg, timeout);
  if (ret != 0) {
    return -EAGAIN;
  }

  return 0;
}
