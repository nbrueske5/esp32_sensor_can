#include "can_sensor_rx_generated.h"

sensor_data_t sensor_can_data;
SemaphoreHandle_t sensor_data_mutex = NULL;

// ----------------------------------------------------------------------
// INTERNAL FORWARD DECLARATIONS
// ----------------------------------------------------------------------
static void handle_sensor_sw_control(const void* msg);
static void handle_sensor_front_imu_lat(const void* msg);
static void handle_sensor_front_imu_tang(const void* msg);
static void handle_sensor_front_imu_vert(const void* msg);
static void handle_sensor_mid_imu_vert(const void* msg);
static void handle_sensor_mid_imu_tang(const void* msg);
static void handle_sensor_mid_imu_lat(const void* msg);
static void handle_sensor_sbg_ekf_altitude(const void* msg);
static void handle_sensor_sbg_gps_position(const void* msg);
static void handle_sensor_sbg_ekf_position(const void* msg);
static void handle_sensor_sbg_ekf_velocity(const void* msg);
static void handle_sensor_sbg_heading(const void* msg);
static void handle_sensor_sbg_gyro(const void* msg);
static void handle_sensor_sbg_roll_yaw_pitch(const void* msg);
static void handle_sensor_sbg_velocity_info(const void* msg);
static void handle_sensor_sbg_acceleration(const void* msg);
static void handle_sensor_sbg_body_vel(const void* msg);
static void handle_sensor_brake_rotor_temp1(const void* msg);
static void handle_sensor_brake_rotor_temp2(const void* msg);
static void handle_sensor_brake_rotor_temp3(const void* msg);
static void handle_sensor_brake_rotor_temp4(const void* msg);
static void handle_sensor_brake_rotor_sensor_temp(const void* msg);
static void handle_sensor_log_driver_inputs(const void* msg);
static void handle_sensor_log_wheel_slip(const void* msg);
static void handle_sensor_log_states(const void* msg);
static void handle_sensor_log_traction_control(const void* msg);
static void handle_sensor_log_battery_limits(const void* msg);
static void handle_sensor_log_torque_limits(const void* msg);
static void handle_sensor_log_motor_power(const void* msg);
static void handle_sensor_log_vehicle_dynamics(const void* msg);
static void handle_sensor_log_torque_vectoring(const void* msg);
static void handle_sensor_log_lap_times(const void* msg);
static void handle_sensor_ecu_battery_tracking(const void* msg);
static void handle_sensor_log_load_transfer(const void* msg);
static void handle_sensor_log_speed_estimation(const void* msg);
static void handle_sensor_log_traction_control_components(const void* msg);

// Static instances for cantools unpacking
static struct sensor_sw_control_t sensor_sw_control_msg;
static struct sensor_front_imu_lat_t sensor_front_imu_lat_msg;
static struct sensor_front_imu_tang_t sensor_front_imu_tang_msg;
static struct sensor_front_imu_vert_t sensor_front_imu_vert_msg;
static struct sensor_mid_imu_vert_t sensor_mid_imu_vert_msg;
static struct sensor_mid_imu_tang_t sensor_mid_imu_tang_msg;
static struct sensor_mid_imu_lat_t sensor_mid_imu_lat_msg;
static struct sensor_sbg_ekf_altitude_t sensor_sbg_ekf_altitude_msg;
static struct sensor_sbg_gps_position_t sensor_sbg_gps_position_msg;
static struct sensor_sbg_ekf_position_t sensor_sbg_ekf_position_msg;
static struct sensor_sbg_ekf_velocity_t sensor_sbg_ekf_velocity_msg;
static struct sensor_sbg_heading_t sensor_sbg_heading_msg;
static struct sensor_sbg_gyro_t sensor_sbg_gyro_msg;
static struct sensor_sbg_roll_yaw_pitch_t sensor_sbg_roll_yaw_pitch_msg;
static struct sensor_sbg_velocity_info_t sensor_sbg_velocity_info_msg;
static struct sensor_sbg_acceleration_t sensor_sbg_acceleration_msg;
static struct sensor_sbg_body_vel_t sensor_sbg_body_vel_msg;
static struct sensor_brake_rotor_temp1_t sensor_brake_rotor_temp1_msg;
static struct sensor_brake_rotor_temp2_t sensor_brake_rotor_temp2_msg;
static struct sensor_brake_rotor_temp3_t sensor_brake_rotor_temp3_msg;
static struct sensor_brake_rotor_temp4_t sensor_brake_rotor_temp4_msg;
static struct sensor_brake_rotor_sensor_temp_t sensor_brake_rotor_sensor_temp_msg;
static struct sensor_log_driver_inputs_t sensor_log_driver_inputs_msg;
static struct sensor_log_wheel_slip_t sensor_log_wheel_slip_msg;
static struct sensor_log_states_t sensor_log_states_msg;
static struct sensor_log_traction_control_t sensor_log_traction_control_msg;
static struct sensor_log_battery_limits_t sensor_log_battery_limits_msg;
static struct sensor_log_torque_limits_t sensor_log_torque_limits_msg;
static struct sensor_log_motor_power_t sensor_log_motor_power_msg;
static struct sensor_log_vehicle_dynamics_t sensor_log_vehicle_dynamics_msg;
static struct sensor_log_torque_vectoring_t sensor_log_torque_vectoring_msg;
static struct sensor_log_lap_times_t sensor_log_lap_times_msg;
static struct sensor_ecu_battery_tracking_t sensor_ecu_battery_tracking_msg;
static struct sensor_log_load_transfer_t sensor_log_load_transfer_msg;
static struct sensor_log_speed_estimation_t sensor_log_speed_estimation_msg;
static struct sensor_log_traction_control_components_t sensor_log_traction_control_components_msg;

typedef struct {
    uint32_t id;
    size_t   sz;
    void    *ptr;
    int    (*unp)(void*, const uint8_t*, size_t);
    void   (*hnd)(const void*);
} msg_desc_t;

// ----------------------------------------------------------------------
// MESSAGE REGISTRY
// ----------------------------------------------------------------------
static const msg_desc_t sensor_registry[] = {
    {
        .id     = SENSOR_SW_CONTROL_FRAME_ID,
        .sz     = SENSOR_SW_CONTROL_LENGTH,
        .ptr    = &sensor_sw_control_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sw_control_unpack,
        .hnd    = handle_sensor_sw_control
    },
    {
        .id     = SENSOR_FRONT_IMU_LAT_FRAME_ID,
        .sz     = SENSOR_FRONT_IMU_LAT_LENGTH,
        .ptr    = &sensor_front_imu_lat_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_front_imu_lat_unpack,
        .hnd    = handle_sensor_front_imu_lat
    },
    {
        .id     = SENSOR_FRONT_IMU_TANG_FRAME_ID,
        .sz     = SENSOR_FRONT_IMU_TANG_LENGTH,
        .ptr    = &sensor_front_imu_tang_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_front_imu_tang_unpack,
        .hnd    = handle_sensor_front_imu_tang
    },
    {
        .id     = SENSOR_FRONT_IMU_VERT_FRAME_ID,
        .sz     = SENSOR_FRONT_IMU_VERT_LENGTH,
        .ptr    = &sensor_front_imu_vert_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_front_imu_vert_unpack,
        .hnd    = handle_sensor_front_imu_vert
    },
    {
        .id     = SENSOR_MID_IMU_VERT_FRAME_ID,
        .sz     = SENSOR_MID_IMU_VERT_LENGTH,
        .ptr    = &sensor_mid_imu_vert_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_mid_imu_vert_unpack,
        .hnd    = handle_sensor_mid_imu_vert
    },
    {
        .id     = SENSOR_MID_IMU_TANG_FRAME_ID,
        .sz     = SENSOR_MID_IMU_TANG_LENGTH,
        .ptr    = &sensor_mid_imu_tang_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_mid_imu_tang_unpack,
        .hnd    = handle_sensor_mid_imu_tang
    },
    {
        .id     = SENSOR_MID_IMU_LAT_FRAME_ID,
        .sz     = SENSOR_MID_IMU_LAT_LENGTH,
        .ptr    = &sensor_mid_imu_lat_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_mid_imu_lat_unpack,
        .hnd    = handle_sensor_mid_imu_lat
    },
    {
        .id     = SENSOR_SBG_EKF_ALTITUDE_FRAME_ID,
        .sz     = SENSOR_SBG_EKF_ALTITUDE_LENGTH,
        .ptr    = &sensor_sbg_ekf_altitude_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_ekf_altitude_unpack,
        .hnd    = handle_sensor_sbg_ekf_altitude
    },
    {
        .id     = SENSOR_SBG_GPS_POSITION_FRAME_ID,
        .sz     = SENSOR_SBG_GPS_POSITION_LENGTH,
        .ptr    = &sensor_sbg_gps_position_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_gps_position_unpack,
        .hnd    = handle_sensor_sbg_gps_position
    },
    {
        .id     = SENSOR_SBG_EKF_POSITION_FRAME_ID,
        .sz     = SENSOR_SBG_EKF_POSITION_LENGTH,
        .ptr    = &sensor_sbg_ekf_position_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_ekf_position_unpack,
        .hnd    = handle_sensor_sbg_ekf_position
    },
    {
        .id     = SENSOR_SBG_EKF_VELOCITY_FRAME_ID,
        .sz     = SENSOR_SBG_EKF_VELOCITY_LENGTH,
        .ptr    = &sensor_sbg_ekf_velocity_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_ekf_velocity_unpack,
        .hnd    = handle_sensor_sbg_ekf_velocity
    },
    {
        .id     = SENSOR_SBG_HEADING_FRAME_ID,
        .sz     = SENSOR_SBG_HEADING_LENGTH,
        .ptr    = &sensor_sbg_heading_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_heading_unpack,
        .hnd    = handle_sensor_sbg_heading
    },
    {
        .id     = SENSOR_SBG_GYRO_FRAME_ID,
        .sz     = SENSOR_SBG_GYRO_LENGTH,
        .ptr    = &sensor_sbg_gyro_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_gyro_unpack,
        .hnd    = handle_sensor_sbg_gyro
    },
    {
        .id     = SENSOR_SBG_ROLL_YAW_PITCH_FRAME_ID,
        .sz     = SENSOR_SBG_ROLL_YAW_PITCH_LENGTH,
        .ptr    = &sensor_sbg_roll_yaw_pitch_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_roll_yaw_pitch_unpack,
        .hnd    = handle_sensor_sbg_roll_yaw_pitch
    },
    {
        .id     = SENSOR_SBG_VELOCITY_INFO_FRAME_ID,
        .sz     = SENSOR_SBG_VELOCITY_INFO_LENGTH,
        .ptr    = &sensor_sbg_velocity_info_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_velocity_info_unpack,
        .hnd    = handle_sensor_sbg_velocity_info
    },
    {
        .id     = SENSOR_SBG_ACCELERATION_FRAME_ID,
        .sz     = SENSOR_SBG_ACCELERATION_LENGTH,
        .ptr    = &sensor_sbg_acceleration_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_acceleration_unpack,
        .hnd    = handle_sensor_sbg_acceleration
    },
    {
        .id     = SENSOR_SBG_BODY_VEL_FRAME_ID,
        .sz     = SENSOR_SBG_BODY_VEL_LENGTH,
        .ptr    = &sensor_sbg_body_vel_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sbg_body_vel_unpack,
        .hnd    = handle_sensor_sbg_body_vel
    },
    {
        .id     = SENSOR_BRAKE_ROTOR_TEMP1_FRAME_ID,
        .sz     = SENSOR_BRAKE_ROTOR_TEMP1_LENGTH,
        .ptr    = &sensor_brake_rotor_temp1_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_brake_rotor_temp1_unpack,
        .hnd    = handle_sensor_brake_rotor_temp1
    },
    {
        .id     = SENSOR_BRAKE_ROTOR_TEMP2_FRAME_ID,
        .sz     = SENSOR_BRAKE_ROTOR_TEMP2_LENGTH,
        .ptr    = &sensor_brake_rotor_temp2_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_brake_rotor_temp2_unpack,
        .hnd    = handle_sensor_brake_rotor_temp2
    },
    {
        .id     = SENSOR_BRAKE_ROTOR_TEMP3_FRAME_ID,
        .sz     = SENSOR_BRAKE_ROTOR_TEMP3_LENGTH,
        .ptr    = &sensor_brake_rotor_temp3_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_brake_rotor_temp3_unpack,
        .hnd    = handle_sensor_brake_rotor_temp3
    },
    {
        .id     = SENSOR_BRAKE_ROTOR_TEMP4_FRAME_ID,
        .sz     = SENSOR_BRAKE_ROTOR_TEMP4_LENGTH,
        .ptr    = &sensor_brake_rotor_temp4_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_brake_rotor_temp4_unpack,
        .hnd    = handle_sensor_brake_rotor_temp4
    },
    {
        .id     = SENSOR_BRAKE_ROTOR_SENSOR_TEMP_FRAME_ID,
        .sz     = SENSOR_BRAKE_ROTOR_SENSOR_TEMP_LENGTH,
        .ptr    = &sensor_brake_rotor_sensor_temp_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_brake_rotor_sensor_temp_unpack,
        .hnd    = handle_sensor_brake_rotor_sensor_temp
    },
    {
        .id     = SENSOR_LOG_DRIVER_INPUTS_FRAME_ID,
        .sz     = SENSOR_LOG_DRIVER_INPUTS_LENGTH,
        .ptr    = &sensor_log_driver_inputs_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_driver_inputs_unpack,
        .hnd    = handle_sensor_log_driver_inputs
    },
    {
        .id     = SENSOR_LOG_WHEEL_SLIP_FRAME_ID,
        .sz     = SENSOR_LOG_WHEEL_SLIP_LENGTH,
        .ptr    = &sensor_log_wheel_slip_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_wheel_slip_unpack,
        .hnd    = handle_sensor_log_wheel_slip
    },
    {
        .id     = SENSOR_LOG_STATES_FRAME_ID,
        .sz     = SENSOR_LOG_STATES_LENGTH,
        .ptr    = &sensor_log_states_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_states_unpack,
        .hnd    = handle_sensor_log_states
    },
    {
        .id     = SENSOR_LOG_TRACTION_CONTROL_FRAME_ID,
        .sz     = SENSOR_LOG_TRACTION_CONTROL_LENGTH,
        .ptr    = &sensor_log_traction_control_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_traction_control_unpack,
        .hnd    = handle_sensor_log_traction_control
    },
    {
        .id     = SENSOR_LOG_BATTERY_LIMITS_FRAME_ID,
        .sz     = SENSOR_LOG_BATTERY_LIMITS_LENGTH,
        .ptr    = &sensor_log_battery_limits_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_battery_limits_unpack,
        .hnd    = handle_sensor_log_battery_limits
    },
    {
        .id     = SENSOR_LOG_TORQUE_LIMITS_FRAME_ID,
        .sz     = SENSOR_LOG_TORQUE_LIMITS_LENGTH,
        .ptr    = &sensor_log_torque_limits_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_torque_limits_unpack,
        .hnd    = handle_sensor_log_torque_limits
    },
    {
        .id     = SENSOR_LOG_MOTOR_POWER_FRAME_ID,
        .sz     = SENSOR_LOG_MOTOR_POWER_LENGTH,
        .ptr    = &sensor_log_motor_power_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_motor_power_unpack,
        .hnd    = handle_sensor_log_motor_power
    },
    {
        .id     = SENSOR_LOG_VEHICLE_DYNAMICS_FRAME_ID,
        .sz     = SENSOR_LOG_VEHICLE_DYNAMICS_LENGTH,
        .ptr    = &sensor_log_vehicle_dynamics_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_vehicle_dynamics_unpack,
        .hnd    = handle_sensor_log_vehicle_dynamics
    },
    {
        .id     = SENSOR_LOG_TORQUE_VECTORING_FRAME_ID,
        .sz     = SENSOR_LOG_TORQUE_VECTORING_LENGTH,
        .ptr    = &sensor_log_torque_vectoring_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_torque_vectoring_unpack,
        .hnd    = handle_sensor_log_torque_vectoring
    },
    {
        .id     = SENSOR_LOG_LAP_TIMES_FRAME_ID,
        .sz     = SENSOR_LOG_LAP_TIMES_LENGTH,
        .ptr    = &sensor_log_lap_times_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_lap_times_unpack,
        .hnd    = handle_sensor_log_lap_times
    },
    {
        .id     = SENSOR_ECU_BATTERY_TRACKING_FRAME_ID,
        .sz     = SENSOR_ECU_BATTERY_TRACKING_LENGTH,
        .ptr    = &sensor_ecu_battery_tracking_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_ecu_battery_tracking_unpack,
        .hnd    = handle_sensor_ecu_battery_tracking
    },
    {
        .id     = SENSOR_LOG_LOAD_TRANSFER_FRAME_ID,
        .sz     = SENSOR_LOG_LOAD_TRANSFER_LENGTH,
        .ptr    = &sensor_log_load_transfer_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_load_transfer_unpack,
        .hnd    = handle_sensor_log_load_transfer
    },
    {
        .id     = SENSOR_LOG_SPEED_ESTIMATION_FRAME_ID,
        .sz     = SENSOR_LOG_SPEED_ESTIMATION_LENGTH,
        .ptr    = &sensor_log_speed_estimation_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_speed_estimation_unpack,
        .hnd    = handle_sensor_log_speed_estimation
    },
    {
        .id     = SENSOR_LOG_TRACTION_CONTROL_COMPONENTS_FRAME_ID,
        .sz     = SENSOR_LOG_TRACTION_CONTROL_COMPONENTS_LENGTH,
        .ptr    = &sensor_log_traction_control_components_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_log_traction_control_components_unpack,
        .hnd    = handle_sensor_log_traction_control_components
    },
};

void sensor_can_data_init(void) {
    if (sensor_data_mutex == NULL) {
        sensor_data_mutex = xSemaphoreCreateMutex();
    }
}

int unpack_sensor_message(uint32_t id, const uint8_t *data, size_t len) {
    for (int i = 0; i < (sizeof(sensor_registry)/sizeof(msg_desc_t)); i++) {
        if (sensor_registry[i].id == id) {
            if (len != sensor_registry[i].sz) return -1;
            if (sensor_registry[i].unp(sensor_registry[i].ptr, data, len) == 0) {
                sensor_registry[i].hnd(sensor_registry[i].ptr);
                return 0;
            }
            return -1;
        }
    }
    return -1;
}

// ----------------------------------------------------------------------
// STATIC HANDLER IMPLEMENTATIONS
// ----------------------------------------------------------------------
static void handle_sensor_sw_control(const void* msg) {
    const struct sensor_sw_control_t* m = (const struct sensor_sw_control_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sw_led_number = sensor_sw_control_sw_led_number_decode(m->sw_led_number);
            sensor_can_data.sw_led_brightness = sensor_sw_control_sw_led_brightness_decode(m->sw_led_brightness);
            sensor_can_data.sw_led_r = sensor_sw_control_sw_led_r_decode(m->sw_led_r);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_front_imu_lat(const void* msg) {
    const struct sensor_front_imu_lat_t* m = (const struct sensor_front_imu_lat_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.imu_front_yaw_rate = sensor_front_imu_lat_imu_front_yaw_rate_decode(m->imu_front_yaw_rate);
            sensor_can_data.imu_front_accel_lat = sensor_front_imu_lat_imu_front_accel_lat_decode(m->imu_front_accel_lat);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_front_imu_tang(const void* msg) {
    const struct sensor_front_imu_tang_t* m = (const struct sensor_front_imu_tang_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.imu_front_roll_rate = sensor_front_imu_tang_imu_front_roll_rate_decode(m->imu_front_roll_rate);
            sensor_can_data.imu_front_accel_tang = sensor_front_imu_tang_imu_front_accel_tang_decode(m->imu_front_accel_tang);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_front_imu_vert(const void* msg) {
    const struct sensor_front_imu_vert_t* m = (const struct sensor_front_imu_vert_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.imu_front_accel_vert = sensor_front_imu_vert_imu_front_accel_vert_decode(m->imu_front_accel_vert);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_mid_imu_vert(const void* msg) {
    const struct sensor_mid_imu_vert_t* m = (const struct sensor_mid_imu_vert_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.imu_mid_accel_vert = sensor_mid_imu_vert_imu_mid_accel_vert_decode(m->imu_mid_accel_vert);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_mid_imu_tang(const void* msg) {
    const struct sensor_mid_imu_tang_t* m = (const struct sensor_mid_imu_tang_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.imu_mid_roll_rate = sensor_mid_imu_tang_imu_mid_roll_rate_decode(m->imu_mid_roll_rate);
            sensor_can_data.imu_mid_accel_tang = sensor_mid_imu_tang_imu_mid_accel_tang_decode(m->imu_mid_accel_tang);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_mid_imu_lat(const void* msg) {
    const struct sensor_mid_imu_lat_t* m = (const struct sensor_mid_imu_lat_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.imu_mid_yaw_rate = sensor_mid_imu_lat_imu_mid_yaw_rate_decode(m->imu_mid_yaw_rate);
            sensor_can_data.imu_mid_accel_lat = sensor_mid_imu_lat_imu_mid_accel_lat_decode(m->imu_mid_accel_lat);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_ekf_altitude(const void* msg) {
    const struct sensor_sbg_ekf_altitude_t* m = (const struct sensor_sbg_ekf_altitude_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_altitude = sensor_sbg_ekf_altitude_sbg_altitude_decode(m->sbg_altitude);
            sensor_can_data.sbg_undulation = sensor_sbg_ekf_altitude_sbg_undulation_decode(m->sbg_undulation);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_gps_position(const void* msg) {
    const struct sensor_sbg_gps_position_t* m = (const struct sensor_sbg_gps_position_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_gps_latitude = sensor_sbg_gps_position_sbg_gps_latitude_decode(m->sbg_gps_latitude);
            sensor_can_data.sbg_gps_longitude = sensor_sbg_gps_position_sbg_gps_longitude_decode(m->sbg_gps_longitude);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_ekf_position(const void* msg) {
    const struct sensor_sbg_ekf_position_t* m = (const struct sensor_sbg_ekf_position_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_ekf_latitude = sensor_sbg_ekf_position_sbg_ekf_latitude_decode(m->sbg_ekf_latitude);
            sensor_can_data.sbg_ekf_longitude = sensor_sbg_ekf_position_sbg_ekf_longitude_decode(m->sbg_ekf_longitude);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_ekf_velocity(const void* msg) {
    const struct sensor_sbg_ekf_velocity_t* m = (const struct sensor_sbg_ekf_velocity_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_velocity_tang = sensor_sbg_ekf_velocity_sbg_velocity_tang_decode(m->sbg_velocity_tang);
            sensor_can_data.sbg_velocity_lat = sensor_sbg_ekf_velocity_sbg_velocity_lat_decode(m->sbg_velocity_lat);
            sensor_can_data.sbg_velocity_vert = sensor_sbg_ekf_velocity_sbg_velocity_vert_decode(m->sbg_velocity_vert);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_heading(const void* msg) {
    const struct sensor_sbg_heading_t* m = (const struct sensor_sbg_heading_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_heading = sensor_sbg_heading_sbg_heading_decode(m->sbg_heading);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_gyro(const void* msg) {
    const struct sensor_sbg_gyro_t* m = (const struct sensor_sbg_gyro_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_yaw_rate_x = sensor_sbg_gyro_sbg_yaw_rate_x_decode(m->sbg_yaw_rate_x);
            sensor_can_data.sbg_yaw_rate_y = sensor_sbg_gyro_sbg_yaw_rate_y_decode(m->sbg_yaw_rate_y);
            sensor_can_data.sbg_yaw_rate_z = sensor_sbg_gyro_sbg_yaw_rate_z_decode(m->sbg_yaw_rate_z);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_roll_yaw_pitch(const void* msg) {
    const struct sensor_sbg_roll_yaw_pitch_t* m = (const struct sensor_sbg_roll_yaw_pitch_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_roll = sensor_sbg_roll_yaw_pitch_sbg_roll_decode(m->sbg_roll);
            sensor_can_data.sbg_pitch = sensor_sbg_roll_yaw_pitch_sbg_pitch_decode(m->sbg_pitch);
            sensor_can_data.sbg_yaw = sensor_sbg_roll_yaw_pitch_sbg_yaw_decode(m->sbg_yaw);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_velocity_info(const void* msg) {
    const struct sensor_sbg_velocity_info_t* m = (const struct sensor_sbg_velocity_info_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_velocity_status = sensor_sbg_velocity_info_sbg_velocity_status_decode(m->sbg_velocity_status);
            sensor_can_data.sbg_velocity_type = sensor_sbg_velocity_info_sbg_velocity_type_decode(m->sbg_velocity_type);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_acceleration(const void* msg) {
    const struct sensor_sbg_acceleration_t* m = (const struct sensor_sbg_acceleration_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sbg_accel_tang = sensor_sbg_acceleration_sbg_accel_tang_decode(m->sbg_accel_tang);
            sensor_can_data.sbg_accel_lat = sensor_sbg_acceleration_sbg_accel_lat_decode(m->sbg_accel_lat);
            sensor_can_data.sbg_accel_vert = sensor_sbg_acceleration_sbg_accel_vert_decode(m->sbg_accel_vert);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_sbg_body_vel(const void* msg) {
    const struct sensor_sbg_body_vel_t* m = (const struct sensor_sbg_body_vel_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.vel_tang = sensor_sbg_body_vel_vel_tang_decode(m->vel_tang);
            sensor_can_data.vel_lat = sensor_sbg_body_vel_vel_lat_decode(m->vel_lat);
            sensor_can_data.vel_vert = sensor_sbg_body_vel_vel_vert_decode(m->vel_vert);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_brake_rotor_temp1(const void* msg) {
    const struct sensor_brake_rotor_temp1_t* m = (const struct sensor_brake_rotor_temp1_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.rotor_temp_1 = sensor_brake_rotor_temp1_rotor_temp_1_decode(m->rotor_temp_1);
            sensor_can_data.rotor_temp_2 = sensor_brake_rotor_temp1_rotor_temp_2_decode(m->rotor_temp_2);
            sensor_can_data.rotor_temp_3 = sensor_brake_rotor_temp1_rotor_temp_3_decode(m->rotor_temp_3);
            sensor_can_data.rotor_temp_4 = sensor_brake_rotor_temp1_rotor_temp_4_decode(m->rotor_temp_4);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_brake_rotor_temp2(const void* msg) {
    const struct sensor_brake_rotor_temp2_t* m = (const struct sensor_brake_rotor_temp2_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.rotor_temp_5 = sensor_brake_rotor_temp2_rotor_temp_5_decode(m->rotor_temp_5);
            sensor_can_data.rotor_temp_6 = sensor_brake_rotor_temp2_rotor_temp_6_decode(m->rotor_temp_6);
            sensor_can_data.rotor_temp_7 = sensor_brake_rotor_temp2_rotor_temp_7_decode(m->rotor_temp_7);
            sensor_can_data.rotor_temp_8 = sensor_brake_rotor_temp2_rotor_temp_8_decode(m->rotor_temp_8);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_brake_rotor_temp3(const void* msg) {
    const struct sensor_brake_rotor_temp3_t* m = (const struct sensor_brake_rotor_temp3_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.rotor_temp_9 = sensor_brake_rotor_temp3_rotor_temp_9_decode(m->rotor_temp_9);
            sensor_can_data.rotor_temp_10 = sensor_brake_rotor_temp3_rotor_temp_10_decode(m->rotor_temp_10);
            sensor_can_data.rotor_temp_11 = sensor_brake_rotor_temp3_rotor_temp_11_decode(m->rotor_temp_11);
            sensor_can_data.rotor_temp_12 = sensor_brake_rotor_temp3_rotor_temp_12_decode(m->rotor_temp_12);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_brake_rotor_temp4(const void* msg) {
    const struct sensor_brake_rotor_temp4_t* m = (const struct sensor_brake_rotor_temp4_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.rotor_temp_13 = sensor_brake_rotor_temp4_rotor_temp_13_decode(m->rotor_temp_13);
            sensor_can_data.rotor_temp_14 = sensor_brake_rotor_temp4_rotor_temp_14_decode(m->rotor_temp_14);
            sensor_can_data.rotor_temp_15 = sensor_brake_rotor_temp4_rotor_temp_15_decode(m->rotor_temp_15);
            sensor_can_data.rotor_temp_16 = sensor_brake_rotor_temp4_rotor_temp_16_decode(m->rotor_temp_16);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_brake_rotor_sensor_temp(const void* msg) {
    const struct sensor_brake_rotor_sensor_temp_t* m = (const struct sensor_brake_rotor_sensor_temp_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.brake_sensor_temp = sensor_brake_rotor_sensor_temp_brake_sensor_temp_decode(m->brake_sensor_temp);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_driver_inputs(const void* msg) {
    const struct sensor_log_driver_inputs_t* m = (const struct sensor_log_driver_inputs_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.brake_pedal_pressure_front = sensor_log_driver_inputs_brake_pedal_pressure_front_decode(m->brake_pedal_pressure_front);
            sensor_can_data.brake_pressure_rear = sensor_log_driver_inputs_brake_pressure_rear_decode(m->brake_pressure_rear);
            sensor_can_data.accel_pedal_position_1 = sensor_log_driver_inputs_accel_pedal_position_1_decode(m->accel_pedal_position_1);
            sensor_can_data.accel_pedal_position_2 = sensor_log_driver_inputs_accel_pedal_position_2_decode(m->accel_pedal_position_2);
            sensor_can_data.accel_pedal_position_arb = sensor_log_driver_inputs_accel_pedal_position_arb_decode(m->accel_pedal_position_arb);
            sensor_can_data.brake_flag = (bool)sensor_log_driver_inputs_brake_flag_decode(m->brake_flag);
            sensor_can_data.tripping_bps = (bool)sensor_log_driver_inputs_tripping_bps_decode(m->tripping_bps);
            sensor_can_data.steering_angle = (bool)sensor_log_driver_inputs_steering_angle_decode(m->steering_angle);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_wheel_slip(const void* msg) {
    const struct sensor_log_wheel_slip_t* m = (const struct sensor_log_wheel_slip_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.wheel_slip_fl = sensor_log_wheel_slip_wheel_slip_fl_decode(m->wheel_slip_fl);
            sensor_can_data.wheel_slip_fr = sensor_log_wheel_slip_wheel_slip_fr_decode(m->wheel_slip_fr);
            sensor_can_data.wheel_slip_rl = sensor_log_wheel_slip_wheel_slip_rl_decode(m->wheel_slip_rl);
            sensor_can_data.wheel_slip_rr = sensor_log_wheel_slip_wheel_slip_rr_decode(m->wheel_slip_rr);
            sensor_can_data.slip_target = sensor_log_wheel_slip_slip_target_decode(m->slip_target);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_states(const void* msg) {
    const struct sensor_log_states_t* m = (const struct sensor_log_states_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.pump_enabled = (bool)sensor_log_states_pump_enabled_decode(m->pump_enabled);
            sensor_can_data.fans_enabled = (bool)sensor_log_states_fans_enabled_decode(m->fans_enabled);
            sensor_can_data.regen_enabled = (bool)sensor_log_states_regen_enabled_decode(m->regen_enabled);
            sensor_can_data.motors_enabled = (bool)sensor_log_states_motors_enabled_decode(m->motors_enabled);
            sensor_can_data.rtd = (bool)sensor_log_states_rtd_decode(m->rtd);
            sensor_can_data.airs = (bool)sensor_log_states_airs_decode(m->airs);
            sensor_can_data.ecu_open_airs = (bool)sensor_log_states_ecu_open_airs_decode(m->ecu_open_airs);
            sensor_can_data.speed_limit_enabled = (bool)sensor_log_states_speed_limit_enabled_decode(m->speed_limit_enabled);
            sensor_can_data.time = sensor_log_states_time_decode(m->time);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_traction_control(const void* msg) {
    const struct sensor_log_traction_control_t* m = (const struct sensor_log_traction_control_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.tc_torque_limit_fl = sensor_log_traction_control_tc_torque_limit_fl_decode(m->tc_torque_limit_fl);
            sensor_can_data.tc_torque_limit_fr = sensor_log_traction_control_tc_torque_limit_fr_decode(m->tc_torque_limit_fr);
            sensor_can_data.tc_torque_limit_rl = sensor_log_traction_control_tc_torque_limit_rl_decode(m->tc_torque_limit_rl);
            sensor_can_data.tc_torque_limit_rr = sensor_log_traction_control_tc_torque_limit_rr_decode(m->tc_torque_limit_rr);
            sensor_can_data.tc_setting = sensor_log_traction_control_tc_setting_decode(m->tc_setting);
            sensor_can_data.tc_active = (bool)sensor_log_traction_control_tc_active_decode(m->tc_active);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_battery_limits(const void* msg) {
    const struct sensor_log_battery_limits_t* m = (const struct sensor_log_battery_limits_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.charge_power_limit = sensor_log_battery_limits_charge_power_limit_decode(m->charge_power_limit);
            sensor_can_data.discharge_power_limit = sensor_log_battery_limits_discharge_power_limit_decode(m->discharge_power_limit);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_torque_limits(const void* msg) {
    const struct sensor_log_torque_limits_t* m = (const struct sensor_log_torque_limits_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.speed_lim_torque_max = sensor_log_torque_limits_speed_lim_torque_max_decode(m->speed_lim_torque_max);
            sensor_can_data.global_torque_max = sensor_log_torque_limits_global_torque_max_decode(m->global_torque_max);
            sensor_can_data.global_torque_min = sensor_log_torque_limits_global_torque_min_decode(m->global_torque_min);
            sensor_can_data.power_torque_max = sensor_log_torque_limits_power_torque_max_decode(m->power_torque_max);
            sensor_can_data.power_torque_min = sensor_log_torque_limits_power_torque_min_decode(m->power_torque_min);
            sensor_can_data.low_speed_torque_min = sensor_log_torque_limits_low_speed_torque_min_decode(m->low_speed_torque_min);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_motor_power(const void* msg) {
    const struct sensor_log_motor_power_t* m = (const struct sensor_log_motor_power_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.motor_power_fl = sensor_log_motor_power_motor_power_fl_decode(m->motor_power_fl);
            sensor_can_data.motor_power_fr = sensor_log_motor_power_motor_power_fr_decode(m->motor_power_fr);
            sensor_can_data.motor_power_rl = sensor_log_motor_power_motor_power_rl_decode(m->motor_power_rl);
            sensor_can_data.motor_power_rr = sensor_log_motor_power_motor_power_rr_decode(m->motor_power_rr);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_vehicle_dynamics(const void* msg) {
    const struct sensor_log_vehicle_dynamics_t* m = (const struct sensor_log_vehicle_dynamics_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.g_ratio = sensor_log_vehicle_dynamics_g_ratio_decode(m->g_ratio);
            sensor_can_data.g_theoretical = sensor_log_vehicle_dynamics_g_theoretical_decode(m->g_theoretical);
            sensor_can_data.g_angle = sensor_log_vehicle_dynamics_g_angle_decode(m->g_angle);
            sensor_can_data.g_actual = sensor_log_vehicle_dynamics_g_actual_decode(m->g_actual);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_torque_vectoring(const void* msg) {
    const struct sensor_log_torque_vectoring_t* m = (const struct sensor_log_torque_vectoring_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.tv_torque_change_fl = sensor_log_torque_vectoring_tv_torque_change_fl_decode(m->tv_torque_change_fl);
            sensor_can_data.tv_torque_change_fr = sensor_log_torque_vectoring_tv_torque_change_fr_decode(m->tv_torque_change_fr);
            sensor_can_data.tv_torque_change_rl = sensor_log_torque_vectoring_tv_torque_change_rl_decode(m->tv_torque_change_rl);
            sensor_can_data.tv_torque_change_rr = sensor_log_torque_vectoring_tv_torque_change_rr_decode(m->tv_torque_change_rr);
            sensor_can_data.tv_active = (bool)sensor_log_torque_vectoring_tv_active_decode(m->tv_active);
            sensor_can_data.tv_setting = sensor_log_torque_vectoring_tv_setting_decode(m->tv_setting);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_lap_times(const void* msg) {
    const struct sensor_log_lap_times_t* m = (const struct sensor_log_lap_times_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.last_lap_time = sensor_log_lap_times_last_lap_time_decode(m->last_lap_time);
            sensor_can_data.lap_counter = sensor_log_lap_times_lap_counter_decode(m->lap_counter);
            sensor_can_data.last_sector_time = sensor_log_lap_times_last_sector_time_decode(m->last_sector_time);
            sensor_can_data.sector_num = sensor_log_lap_times_sector_num_decode(m->sector_num);
            sensor_can_data.lap_distance = sensor_log_lap_times_lap_distance_decode(m->lap_distance);
            sensor_can_data.sector_distance = sensor_log_lap_times_sector_distance_decode(m->sector_distance);
            sensor_can_data.lap_timer_trigger = (bool)sensor_log_lap_times_lap_timer_trigger_decode(m->lap_timer_trigger);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_ecu_battery_tracking(const void* msg) {
    const struct sensor_ecu_battery_tracking_t* m = (const struct sensor_ecu_battery_tracking_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.ecu_last_known_soc = sensor_ecu_battery_tracking_ecu_last_known_soc_decode(m->ecu_last_known_soc);
            sensor_can_data.ecu_soc = sensor_ecu_battery_tracking_ecu_soc_decode(m->ecu_soc);
            sensor_can_data.total_energy_used = sensor_ecu_battery_tracking_total_energy_used_decode(m->total_energy_used);
            sensor_can_data.ecu_soc_est_state = sensor_ecu_battery_tracking_ecu_soc_est_state_decode(m->ecu_soc_est_state);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_load_transfer(const void* msg) {
    const struct sensor_log_load_transfer_t* m = (const struct sensor_log_load_transfer_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.normal_force_fl = sensor_log_load_transfer_normal_force_fl_decode(m->normal_force_fl);
            sensor_can_data.normal_force_fr = sensor_log_load_transfer_normal_force_fr_decode(m->normal_force_fr);
            sensor_can_data.normal_force_rl = sensor_log_load_transfer_normal_force_rl_decode(m->normal_force_rl);
            sensor_can_data.normal_force_rr = sensor_log_load_transfer_normal_force_rr_decode(m->normal_force_rr);
            sensor_can_data.downforce = sensor_log_load_transfer_downforce_decode(m->downforce);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_speed_estimation(const void* msg) {
    const struct sensor_log_speed_estimation_t* m = (const struct sensor_log_speed_estimation_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.est_speed_fl = sensor_log_speed_estimation_est_speed_fl_decode(m->est_speed_fl);
            sensor_can_data.est_speed_fr = sensor_log_speed_estimation_est_speed_fr_decode(m->est_speed_fr);
            sensor_can_data.est_speed_rl = sensor_log_speed_estimation_est_speed_rl_decode(m->est_speed_rl);
            sensor_can_data.est_speed_rr = sensor_log_speed_estimation_est_speed_rr_decode(m->est_speed_rr);
            sensor_can_data.est_speed_vehicle = sensor_log_speed_estimation_est_speed_vehicle_decode(m->est_speed_vehicle);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}

static void handle_sensor_log_traction_control_components(const void* msg) {
    const struct sensor_log_traction_control_components_t* m = (const struct sensor_log_traction_control_components_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.tc_fb_fl = sensor_log_traction_control_components_tc_fb_fl_decode(m->tc_fb_fl);
            sensor_can_data.tc_fb_fr = sensor_log_traction_control_components_tc_fb_fr_decode(m->tc_fb_fr);
            sensor_can_data.tc_fb_rl = sensor_log_traction_control_components_tc_fb_rl_decode(m->tc_fb_rl);
            sensor_can_data.tc_fb_rr = sensor_log_traction_control_components_tc_fb_rr_decode(m->tc_fb_rr);
            sensor_can_data.tc_ff_fl = sensor_log_traction_control_components_tc_ff_fl_decode(m->tc_ff_fl);
            sensor_can_data.tc_ff_fr = sensor_log_traction_control_components_tc_ff_fr_decode(m->tc_ff_fr);
            sensor_can_data.tc_ff_rl = sensor_log_traction_control_components_tc_ff_rl_decode(m->tc_ff_rl);
            sensor_can_data.tc_ff_rr = sensor_log_traction_control_components_tc_ff_rr_decode(m->tc_ff_rr);
            xSemaphoreGive(sensor_data_mutex);
        }
    }
}
