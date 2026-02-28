idf_component_register(SRCS "main.c" "twai.c" "../components/CAN_handler/CAN_generated/can_sensor_rx_generated.c"
                    REQUIRES driver
                    INCLUDE_DIRS ".")
