idf_component_register(SRCS "main.c" "twai.c" "can_sensor_rx_generated.c" "can_sensor_tx_generated.c" "sensor.c"
                    REQUIRES driver
                    INCLUDE_DIRS ".")
