#include "can_sensor_tx_generated.h"
#include "sensor.h"

void can_tx_sensor_sw_control(uint16_t sw_led_number, uint16_t sw_led_brightness, uint16_t sw_led_r, uint16_t sw_led_g, uint16_t sw_led_b) {
    struct sensor_sw_control_t m;

    m.sw_led_number = sensor_sw_control_sw_led_number_encode((float)sw_led_number);
    m.sw_led_brightness = sensor_sw_control_sw_led_brightness_encode((float)sw_led_brightness);
    m.sw_led_r = sensor_sw_control_sw_led_r_encode((float)sw_led_r);
    m.sw_led_g = sensor_sw_control_sw_led_g_encode((float)sw_led_g);
    m.sw_led_b = sensor_sw_control_sw_led_b_encode((float)sw_led_b);

    uint8_t d[SENSOR_SW_CONTROL_LENGTH];
    sensor_sw_control_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_SW_CONTROL_FRAME_ID, d, SENSOR_SW_CONTROL_LENGTH);
}

void can_tx_sensor_log_driver_inputs(float brake_pedal_pressure_front, float brake_pressure_rear, float accel_pedal_position_1, float accel_pedal_position_2, float accel_pedal_position_arb, bool brake_flag, bool tripping_bps, bool steering_angle) {
    struct sensor_log_driver_inputs_t m;

    m.brake_pedal_pressure_front = sensor_log_driver_inputs_brake_pedal_pressure_front_encode((float)brake_pedal_pressure_front);
    m.brake_pressure_rear = sensor_log_driver_inputs_brake_pressure_rear_encode((float)brake_pressure_rear);
    m.accel_pedal_position_1 = sensor_log_driver_inputs_accel_pedal_position_1_encode((float)accel_pedal_position_1);
    m.accel_pedal_position_2 = sensor_log_driver_inputs_accel_pedal_position_2_encode((float)accel_pedal_position_2);
    m.accel_pedal_position_arb = sensor_log_driver_inputs_accel_pedal_position_arb_encode((float)accel_pedal_position_arb);
    m.brake_flag = sensor_log_driver_inputs_brake_flag_encode((float)brake_flag);
    m.tripping_bps = sensor_log_driver_inputs_tripping_bps_encode((float)tripping_bps);
    m.steering_angle = sensor_log_driver_inputs_steering_angle_encode((float)steering_angle);

    uint8_t d[SENSOR_LOG_DRIVER_INPUTS_LENGTH];
    sensor_log_driver_inputs_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_DRIVER_INPUTS_FRAME_ID, d, SENSOR_LOG_DRIVER_INPUTS_LENGTH);
}

void can_tx_sensor_log_wheel_slip(float wheel_slip_fl, float wheel_slip_fr, float wheel_slip_rl, float wheel_slip_rr, float slip_target) {
    struct sensor_log_wheel_slip_t m;

    m.wheel_slip_fl = sensor_log_wheel_slip_wheel_slip_fl_encode((float)wheel_slip_fl);
    m.wheel_slip_fr = sensor_log_wheel_slip_wheel_slip_fr_encode((float)wheel_slip_fr);
    m.wheel_slip_rl = sensor_log_wheel_slip_wheel_slip_rl_encode((float)wheel_slip_rl);
    m.wheel_slip_rr = sensor_log_wheel_slip_wheel_slip_rr_encode((float)wheel_slip_rr);
    m.slip_target = sensor_log_wheel_slip_slip_target_encode((float)slip_target);

    uint8_t d[SENSOR_LOG_WHEEL_SLIP_LENGTH];
    sensor_log_wheel_slip_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_WHEEL_SLIP_FRAME_ID, d, SENSOR_LOG_WHEEL_SLIP_LENGTH);
}

void can_tx_sensor_log_states(bool pump_enabled, bool fans_enabled, bool regen_enabled, bool motors_enabled, bool rtd, bool airs, bool ecu_open_airs, bool speed_limit_enabled, float time) {
    struct sensor_log_states_t m;

    m.pump_enabled = sensor_log_states_pump_enabled_encode((float)pump_enabled);
    m.fans_enabled = sensor_log_states_fans_enabled_encode((float)fans_enabled);
    m.regen_enabled = sensor_log_states_regen_enabled_encode((float)regen_enabled);
    m.motors_enabled = sensor_log_states_motors_enabled_encode((float)motors_enabled);
    m.rtd = sensor_log_states_rtd_encode((float)rtd);
    m.airs = sensor_log_states_airs_encode((float)airs);
    m.ecu_open_airs = sensor_log_states_ecu_open_airs_encode((float)ecu_open_airs);
    m.speed_limit_enabled = sensor_log_states_speed_limit_enabled_encode((float)speed_limit_enabled);
    m.time = sensor_log_states_time_encode((float)time);

    uint8_t d[SENSOR_LOG_STATES_LENGTH];
    sensor_log_states_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_STATES_FRAME_ID, d, SENSOR_LOG_STATES_LENGTH);
}

void can_tx_sensor_log_traction_control(float tc_torque_limit_fl, float tc_torque_limit_fr, float tc_torque_limit_rl, float tc_torque_limit_rr, uint16_t tc_setting, bool tc_active) {
    struct sensor_log_traction_control_t m;

    m.tc_torque_limit_fl = sensor_log_traction_control_tc_torque_limit_fl_encode((float)tc_torque_limit_fl);
    m.tc_torque_limit_fr = sensor_log_traction_control_tc_torque_limit_fr_encode((float)tc_torque_limit_fr);
    m.tc_torque_limit_rl = sensor_log_traction_control_tc_torque_limit_rl_encode((float)tc_torque_limit_rl);
    m.tc_torque_limit_rr = sensor_log_traction_control_tc_torque_limit_rr_encode((float)tc_torque_limit_rr);
    m.tc_setting = sensor_log_traction_control_tc_setting_encode((float)tc_setting);
    m.tc_active = sensor_log_traction_control_tc_active_encode((float)tc_active);

    uint8_t d[SENSOR_LOG_TRACTION_CONTROL_LENGTH];
    sensor_log_traction_control_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_TRACTION_CONTROL_FRAME_ID, d, SENSOR_LOG_TRACTION_CONTROL_LENGTH);
}

void can_tx_sensor_log_battery_limits(float charge_power_limit, float discharge_power_limit) {
    struct sensor_log_battery_limits_t m;

    m.charge_power_limit = sensor_log_battery_limits_charge_power_limit_encode((float)charge_power_limit);
    m.discharge_power_limit = sensor_log_battery_limits_discharge_power_limit_encode((float)discharge_power_limit);

    uint8_t d[SENSOR_LOG_BATTERY_LIMITS_LENGTH];
    sensor_log_battery_limits_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_BATTERY_LIMITS_FRAME_ID, d, SENSOR_LOG_BATTERY_LIMITS_LENGTH);
}

void can_tx_sensor_log_torque_limits(float speed_lim_torque_max, float global_torque_max, float global_torque_min, float power_torque_max, float power_torque_min, float low_speed_torque_min) {
    struct sensor_log_torque_limits_t m;

    m.speed_lim_torque_max = sensor_log_torque_limits_speed_lim_torque_max_encode((float)speed_lim_torque_max);
    m.global_torque_max = sensor_log_torque_limits_global_torque_max_encode((float)global_torque_max);
    m.global_torque_min = sensor_log_torque_limits_global_torque_min_encode((float)global_torque_min);
    m.power_torque_max = sensor_log_torque_limits_power_torque_max_encode((float)power_torque_max);
    m.power_torque_min = sensor_log_torque_limits_power_torque_min_encode((float)power_torque_min);
    m.low_speed_torque_min = sensor_log_torque_limits_low_speed_torque_min_encode((float)low_speed_torque_min);

    uint8_t d[SENSOR_LOG_TORQUE_LIMITS_LENGTH];
    sensor_log_torque_limits_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_TORQUE_LIMITS_FRAME_ID, d, SENSOR_LOG_TORQUE_LIMITS_LENGTH);
}

void can_tx_sensor_log_motor_power(float motor_power_fl, float motor_power_fr, float motor_power_rl, float motor_power_rr) {
    struct sensor_log_motor_power_t m;

    m.motor_power_fl = sensor_log_motor_power_motor_power_fl_encode((float)motor_power_fl);
    m.motor_power_fr = sensor_log_motor_power_motor_power_fr_encode((float)motor_power_fr);
    m.motor_power_rl = sensor_log_motor_power_motor_power_rl_encode((float)motor_power_rl);
    m.motor_power_rr = sensor_log_motor_power_motor_power_rr_encode((float)motor_power_rr);

    uint8_t d[SENSOR_LOG_MOTOR_POWER_LENGTH];
    sensor_log_motor_power_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_MOTOR_POWER_FRAME_ID, d, SENSOR_LOG_MOTOR_POWER_LENGTH);
}

void can_tx_sensor_log_vehicle_dynamics(float g_ratio, float g_theoretical, float g_angle, float g_actual) {
    struct sensor_log_vehicle_dynamics_t m;

    m.g_ratio = sensor_log_vehicle_dynamics_g_ratio_encode((float)g_ratio);
    m.g_theoretical = sensor_log_vehicle_dynamics_g_theoretical_encode((float)g_theoretical);
    m.g_angle = sensor_log_vehicle_dynamics_g_angle_encode((float)g_angle);
    m.g_actual = sensor_log_vehicle_dynamics_g_actual_encode((float)g_actual);

    uint8_t d[SENSOR_LOG_VEHICLE_DYNAMICS_LENGTH];
    sensor_log_vehicle_dynamics_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_VEHICLE_DYNAMICS_FRAME_ID, d, SENSOR_LOG_VEHICLE_DYNAMICS_LENGTH);
}

void can_tx_sensor_log_torque_vectoring(float tv_torque_change_fl, float tv_torque_change_fr, float tv_torque_change_rl, float tv_torque_change_rr, bool tv_active, uint16_t tv_setting) {
    struct sensor_log_torque_vectoring_t m;

    m.tv_torque_change_fl = sensor_log_torque_vectoring_tv_torque_change_fl_encode((float)tv_torque_change_fl);
    m.tv_torque_change_fr = sensor_log_torque_vectoring_tv_torque_change_fr_encode((float)tv_torque_change_fr);
    m.tv_torque_change_rl = sensor_log_torque_vectoring_tv_torque_change_rl_encode((float)tv_torque_change_rl);
    m.tv_torque_change_rr = sensor_log_torque_vectoring_tv_torque_change_rr_encode((float)tv_torque_change_rr);
    m.tv_active = sensor_log_torque_vectoring_tv_active_encode((float)tv_active);
    m.tv_setting = sensor_log_torque_vectoring_tv_setting_encode((float)tv_setting);

    uint8_t d[SENSOR_LOG_TORQUE_VECTORING_LENGTH];
    sensor_log_torque_vectoring_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_TORQUE_VECTORING_FRAME_ID, d, SENSOR_LOG_TORQUE_VECTORING_LENGTH);
}

void can_tx_sensor_log_lap_times(float last_lap_time, uint16_t lap_counter, float last_sector_time, uint16_t sector_num, float lap_distance, float sector_distance, bool lap_timer_trigger) {
    struct sensor_log_lap_times_t m;

    m.last_lap_time = sensor_log_lap_times_last_lap_time_encode((float)last_lap_time);
    m.lap_counter = sensor_log_lap_times_lap_counter_encode((float)lap_counter);
    m.last_sector_time = sensor_log_lap_times_last_sector_time_encode((float)last_sector_time);
    m.sector_num = sensor_log_lap_times_sector_num_encode((float)sector_num);
    m.lap_distance = sensor_log_lap_times_lap_distance_encode((float)lap_distance);
    m.sector_distance = sensor_log_lap_times_sector_distance_encode((float)sector_distance);
    m.lap_timer_trigger = sensor_log_lap_times_lap_timer_trigger_encode((float)lap_timer_trigger);

    uint8_t d[SENSOR_LOG_LAP_TIMES_LENGTH];
    sensor_log_lap_times_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_LAP_TIMES_FRAME_ID, d, SENSOR_LOG_LAP_TIMES_LENGTH);
}

void can_tx_sensor_ecu_battery_tracking(float ecu_last_known_soc, float ecu_soc, uint16_t total_energy_used, uint16_t ecu_soc_est_state) {
    struct sensor_ecu_battery_tracking_t m;

    m.ecu_last_known_soc = sensor_ecu_battery_tracking_ecu_last_known_soc_encode((float)ecu_last_known_soc);
    m.ecu_soc = sensor_ecu_battery_tracking_ecu_soc_encode((float)ecu_soc);
    m.total_energy_used = sensor_ecu_battery_tracking_total_energy_used_encode((float)total_energy_used);
    m.ecu_soc_est_state = sensor_ecu_battery_tracking_ecu_soc_est_state_encode((float)ecu_soc_est_state);

    uint8_t d[SENSOR_ECU_BATTERY_TRACKING_LENGTH];
    sensor_ecu_battery_tracking_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_ECU_BATTERY_TRACKING_FRAME_ID, d, SENSOR_ECU_BATTERY_TRACKING_LENGTH);
}

void can_tx_sensor_log_load_transfer(uint16_t normal_force_fl, uint16_t normal_force_fr, uint16_t normal_force_rl, uint16_t normal_force_rr, uint16_t downforce) {
    struct sensor_log_load_transfer_t m;

    m.normal_force_fl = sensor_log_load_transfer_normal_force_fl_encode((float)normal_force_fl);
    m.normal_force_fr = sensor_log_load_transfer_normal_force_fr_encode((float)normal_force_fr);
    m.normal_force_rl = sensor_log_load_transfer_normal_force_rl_encode((float)normal_force_rl);
    m.normal_force_rr = sensor_log_load_transfer_normal_force_rr_encode((float)normal_force_rr);
    m.downforce = sensor_log_load_transfer_downforce_encode((float)downforce);

    uint8_t d[SENSOR_LOG_LOAD_TRANSFER_LENGTH];
    sensor_log_load_transfer_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_LOAD_TRANSFER_FRAME_ID, d, SENSOR_LOG_LOAD_TRANSFER_LENGTH);
}

void can_tx_sensor_log_speed_estimation(uint16_t est_speed_fl, uint16_t est_speed_fr, uint16_t est_speed_rl, uint16_t est_speed_rr, float est_speed_vehicle) {
    struct sensor_log_speed_estimation_t m;

    m.est_speed_fl = sensor_log_speed_estimation_est_speed_fl_encode((float)est_speed_fl);
    m.est_speed_fr = sensor_log_speed_estimation_est_speed_fr_encode((float)est_speed_fr);
    m.est_speed_rl = sensor_log_speed_estimation_est_speed_rl_encode((float)est_speed_rl);
    m.est_speed_rr = sensor_log_speed_estimation_est_speed_rr_encode((float)est_speed_rr);
    m.est_speed_vehicle = sensor_log_speed_estimation_est_speed_vehicle_encode((float)est_speed_vehicle);

    uint8_t d[SENSOR_LOG_SPEED_ESTIMATION_LENGTH];
    sensor_log_speed_estimation_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_SPEED_ESTIMATION_FRAME_ID, d, SENSOR_LOG_SPEED_ESTIMATION_LENGTH);
}

void can_tx_sensor_log_traction_control_components(float tc_fb_fl, float tc_fb_fr, float tc_fb_rl, float tc_fb_rr, float tc_ff_fl, float tc_ff_fr, float tc_ff_rl, float tc_ff_rr) {
    struct sensor_log_traction_control_components_t m;

    m.tc_fb_fl = sensor_log_traction_control_components_tc_fb_fl_encode((float)tc_fb_fl);
    m.tc_fb_fr = sensor_log_traction_control_components_tc_fb_fr_encode((float)tc_fb_fr);
    m.tc_fb_rl = sensor_log_traction_control_components_tc_fb_rl_encode((float)tc_fb_rl);
    m.tc_fb_rr = sensor_log_traction_control_components_tc_fb_rr_encode((float)tc_fb_rr);
    m.tc_ff_fl = sensor_log_traction_control_components_tc_ff_fl_encode((float)tc_ff_fl);
    m.tc_ff_fr = sensor_log_traction_control_components_tc_ff_fr_encode((float)tc_ff_fr);
    m.tc_ff_rl = sensor_log_traction_control_components_tc_ff_rl_encode((float)tc_ff_rl);
    m.tc_ff_rr = sensor_log_traction_control_components_tc_ff_rr_encode((float)tc_ff_rr);

    uint8_t d[SENSOR_LOG_TRACTION_CONTROL_COMPONENTS_LENGTH];
    sensor_log_traction_control_components_pack(d, &m, sizeof(d));
    send_sensor_can_message(SENSOR_LOG_TRACTION_CONTROL_COMPONENTS_FRAME_ID, d, SENSOR_LOG_TRACTION_CONTROL_COMPONENTS_LENGTH);
}