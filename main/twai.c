#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "can_sensor_rx_generated.h"
#include "can_sensor_tx_generated.h"

twai_node_handle_t node_hdl = NULL;
QueueHandle_t rxQueue_hdl = NULL;
TaskHandle_t twai_rx_queue_hdl = NULL;

static const char *TAG = "TWAI";

/**
 * Task responsible for taking twai frames from the rxQueue and unpacking them using the generated sensor can files
 */
static void twai_rx_task(void *param) {
    twai_message_t message_frame;
    uint8_t message[8] = {0};
    for(;;) {
        if (xQueueReceive(rxQueue_hdl, &message_frame, pdMS_TO_TICKS(20)) == pdTRUE) {
            // unpack the message
            if (unpack_sensor_message(message_frame.identifier, message_frame.data, message_frame.data_length_code) != 0) {
                ESP_LOGE(TAG, "Failed unpacking frame from queue: %d", message_frame.identifier);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * Twai RX callback function. Takes the twai frame, stores it into a twai message and adds it to the rxQueue
 */
static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        // create the message to add to queue
        twai_message_t msg = {
            .identifier = rx_frame.header.id,
            .data_length_code = rx_frame.header.dlc,
        };
        memcpy(msg.data, recv_buff, rx_frame.header.dlc);
        if (xQueueSendFromISR(rxQueue_hdl, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
            //TODO: Queue was full
        }
    }
    return xHigherPriorityTaskWoken;
}

/**
 * Initialize the node and it's associated RX message queue
 */
void twai_init_node(void) {
    // initialize twai node
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = 4,             // TWAI TX GPIO pin
        .io_cfg.rx = 5,             // TWAI RX GPIO pin
        .bit_timing.bitrate = 1000000,  // 1Mb
        .tx_queue_depth = 5,
        .flags = {
            //to enable self testing: 
            //.enable_loopback = 1,
            //.enable_self_test = 1,
        }
    };

    // Create a new TWAI controller driver instance
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));

    // configure receive filter
    twai_mask_filter_config_t mfilter_cfg = {
    .id = 0,    /**< Single base ID for filtering */
    .mask = 0,      /**< Mask to determine the matching bits (1 = match bit, 0 = any bit) */
    .is_ext = false,    // Only use Standard ID
    };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_hdl, 0, &mfilter_cfg));   // Configure on filter 0

    //register receive event callback
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    
    rxQueue_hdl = xQueueCreate(20, sizeof(twai_message_t));  //create the queue for rx

    // Start the TWAI controller
    ESP_ERROR_CHECK(twai_node_enable(node_hdl)); 
    ESP_LOGI(TAG, "created and started the TWAI controller");

    sensor_can_data_init();
    
    // create task to free twai frames from the queue
    xTaskCreate(
        twai_rx_task,
        "twai receive from queue",
        5000, // stack size -> experimental
        NULL, // no params
        4, //PRIORITY -> TODO
        &twai_rx_queue_hdl
    );
}

int send_sensor_can_message(uint32_t id, uint8_t* tx_buff, uint8_t tx_buff_size) {
    // error checks
    if (tx_buff_size > 8 || id > 0x1FFFFFFF) {
        return ESP_ERR_INVALID_SIZE;
    };
    if (tx_buff_size > 0 && tx_buff == NULL) {
        return ESP_ERR_INVALID_ARG;
    };

    // build and transmit the message
    twai_frame_t tx_msg = {
        .header.id = id, //message id
        .header.ide = (id > 0x7FF), //use extended id if id > 11 bits
        .header.dlc = tx_buff_size, //message data length code (ESP32 does not support FD format)
        .buffer = tx_buff,
    };

    return twai_node_transmit(node_hdl, &tx_msg, 0);  // Timeout = 0: returns immediately if queue is full
}


