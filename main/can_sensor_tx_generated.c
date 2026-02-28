#include "can_sensor_tx_generated.h"
#include "sensor.h"

void can_tx_sensor_sw_inputs(bool sw_button1, bool sw_button2, bool sw_joystick_center, bool sw_joystick_down, bool sw_joystick_left, bool sw_joystick_right, bool sw_joystick_up, uint16_t sw_dial_left, uint16_t sw_dial_right) {
    struct sensor_sw_inputs_t m;

    m.sw_button1 = sensor_sw_inputs_sw_button1_encode((float)sw_button1);
    m.sw_button2 = sensor_sw_inputs_sw_button2_encode((float)sw_button2);
    m.sw_joystick_center = sensor_sw_inputs_sw_joystick_center_encode((float)sw_joystick_center);
    m.sw_joystick_down = sensor_sw_inputs_sw_joystick_down_encode((float)sw_joystick_down);
    m.sw_joystick_left = sensor_sw_inputs_sw_joystick_left_encode((float)sw_joystick_left);
    m.sw_joystick_right = sensor_sw_inputs_sw_joystick_right_encode((float)sw_joystick_right);
    m.sw_joystick_up = sensor_sw_inputs_sw_joystick_up_encode((float)sw_joystick_up);
    m.sw_dial_left = sensor_sw_inputs_sw_dial_left_encode((float)sw_dial_left);
    m.sw_dial_right = sensor_sw_inputs_sw_dial_right_encode((float)sw_dial_right);

    uint8_t d[SENSOR_SW_INPUTS_LENGTH];
    sensor_sw_inputs_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_SW_INPUTS_FRAME_ID, d, SENSOR_SW_INPUTS_LENGTH);
}