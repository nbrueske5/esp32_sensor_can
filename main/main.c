#include "twai.h"
#include "freertos/task.h"

void app_main(void) {
    twai_init_node();
    while (true) {
        vTaskDelay(20);
    }
}