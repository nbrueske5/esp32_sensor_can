#include "can_sensor_rx_generated.h"

sensor_data_t sensor_can_data;
SemaphoreHandle_t sensor_data_mutex = NULL;

// ----------------------------------------------------------------------
// INTERNAL FORWARD DECLARATIONS
// ----------------------------------------------------------------------
static void handle_sensor_sw_inputs(const void* msg);
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

// Static instances for cantools unpacking
static struct sensor_sw_inputs_t sensor_sw_inputs_msg;
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
        .id     = SENSOR_SW_INPUTS_FRAME_ID,
        .sz     = SENSOR_SW_INPUTS_LENGTH,
        .ptr    = &sensor_sw_inputs_msg,
        .unp    = (int (*)(void*, const uint8_t*, size_t))sensor_sw_inputs_unpack,
        .hnd    = handle_sensor_sw_inputs
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
static void handle_sensor_sw_inputs(const void* msg) {
    const struct sensor_sw_inputs_t* m = (const struct sensor_sw_inputs_t*)msg;

    if (sensor_data_mutex != NULL) {
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_can_data.sw_button1 = (bool)sensor_sw_inputs_sw_button1_decode(m->sw_button1);
            sensor_can_data.sw_button2 = (bool)sensor_sw_inputs_sw_button2_decode(m->sw_button2);
            sensor_can_data.sw_joystick_center = (bool)sensor_sw_inputs_sw_joystick_center_decode(m->sw_joystick_center);
            sensor_can_data.sw_joystick_down = (bool)sensor_sw_inputs_sw_joystick_down_decode(m->sw_joystick_down);
            sensor_can_data.sw_joystick_left = (bool)sensor_sw_inputs_sw_joystick_left_decode(m->sw_joystick_left);
            sensor_can_data.sw_joystick_right = (bool)sensor_sw_inputs_sw_joystick_right_decode(m->sw_joystick_right);
            sensor_can_data.sw_joystick_up = (bool)sensor_sw_inputs_sw_joystick_up_decode(m->sw_joystick_up);
            sensor_can_data.sw_dial_left = sensor_sw_inputs_sw_dial_left_decode(m->sw_dial_left);
            sensor_can_data.sw_dial_right = sensor_sw_inputs_sw_dial_right_decode(m->sw_dial_right);
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
