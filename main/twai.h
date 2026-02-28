#ifndef TWAI_H
#define TWAI_H

#include <esp_err.h>
#include <esp_twai_types.h>
#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>

extern twai_node_handle_t node_hdl;
extern QueueHandle_t rxQueue_hdl;

void twai_init_node(void);
int send_sensor_can_message(uint32_t can_id, uint8_t *data, uint8_t data_len);

#endif // TWAI_WRAP_H