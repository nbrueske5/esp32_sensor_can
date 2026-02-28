#ifndef CAN_SENSOR_RX_GENERATED_H
#define CAN_SENSOR_RX_GENERATED_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sensor.h"

// ----------------------------------------------------------------------
// SENSOR BUS DATA STRUCTURE
// ----------------------------------------------------------------------
typedef struct {
    bool       sw_button1;
    bool       sw_button2;
    bool       sw_joystick_center;
    bool       sw_joystick_down;
    bool       sw_joystick_left;
    bool       sw_joystick_right;
    bool       sw_joystick_up;
    float      sw_dial_left;
    float      sw_dial_right;
    float      imu_front_yaw_rate;
    float      imu_front_accel_lat;
    float      imu_front_roll_rate;
    float      imu_front_accel_tang;
    float      imu_front_accel_vert;
    float      imu_mid_accel_vert;
    float      imu_mid_roll_rate;
    float      imu_mid_accel_tang;
    float      imu_mid_yaw_rate;
    float      imu_mid_accel_lat;
    float      sbg_altitude;
    float      sbg_undulation;
    float      sbg_gps_latitude;
    float      sbg_gps_longitude;
    float      sbg_ekf_latitude;
    float      sbg_ekf_longitude;
    float      sbg_velocity_tang;
    float      sbg_velocity_lat;
    float      sbg_velocity_vert;
    float      sbg_heading;
    float      sbg_yaw_rate_x;
    float      sbg_yaw_rate_y;
    float      sbg_yaw_rate_z;
    float      sbg_roll;
    float      sbg_pitch;
    float      sbg_yaw;
    float      sbg_velocity_status;
    float      sbg_velocity_type;
    float      sbg_accel_tang;
    float      sbg_accel_lat;
    float      sbg_accel_vert;
    float      vel_tang;
    float      vel_lat;
    float      vel_vert;
} sensor_data_t;

extern sensor_data_t sensor_can_data;
extern SemaphoreHandle_t sensor_data_mutex;

// Public API
void sensor_can_data_init(void);
int unpack_sensor_message(uint32_t id, const uint8_t *data, size_t len);

#endif // CAN_SENSOR_RX_GENERATED_H