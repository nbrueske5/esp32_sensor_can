#ifndef CAN_SENSOR_TX_GENERATED_H
#define CAN_SENSOR_TX_GENERATED_H

#include <stdint.h>
#include <stdbool.h>
#include "CANTX.h"

void can_tx_sensor_sw_control(uint16_t sw_led_number, uint16_t sw_led_brightness, uint16_t sw_led_r, uint16_t sw_led_g, uint16_t sw_led_b);

void can_tx_sensor_log_driver_inputs(float brake_pedal_pressure_front, float brake_pressure_rear, float accel_pedal_position_1, float accel_pedal_position_2, float accel_pedal_position_arb, bool brake_flag, bool tripping_bps, bool steering_angle);

void can_tx_sensor_log_wheel_slip(float wheel_slip_fl, float wheel_slip_fr, float wheel_slip_rl, float wheel_slip_rr, float slip_target);

void can_tx_sensor_log_states(bool pump_enabled, bool fans_enabled, bool regen_enabled, bool motors_enabled, bool rtd, bool airs, bool ecu_open_airs, bool speed_limit_enabled, float time);

void can_tx_sensor_log_traction_control(float tc_torque_limit_fl, float tc_torque_limit_fr, float tc_torque_limit_rl, float tc_torque_limit_rr, uint16_t tc_setting, bool tc_active);

void can_tx_sensor_log_battery_limits(float charge_power_limit, float discharge_power_limit);

void can_tx_sensor_log_torque_limits(float speed_lim_torque_max, float global_torque_max, float global_torque_min, float power_torque_max, float power_torque_min, float low_speed_torque_min);

void can_tx_sensor_log_motor_power(float motor_power_fl, float motor_power_fr, float motor_power_rl, float motor_power_rr);

void can_tx_sensor_log_vehicle_dynamics(float g_ratio, float g_theoretical, float g_angle, float g_actual);

void can_tx_sensor_log_torque_vectoring(float tv_torque_change_fl, float tv_torque_change_fr, float tv_torque_change_rl, float tv_torque_change_rr, bool tv_active, uint16_t tv_setting);

void can_tx_sensor_log_lap_times(float last_lap_time, uint16_t lap_counter, float last_sector_time, uint16_t sector_num, float lap_distance, float sector_distance, bool lap_timer_trigger);

void can_tx_sensor_ecu_battery_tracking(float ecu_last_known_soc, float ecu_soc, uint16_t total_energy_used, uint16_t ecu_soc_est_state);

void can_tx_sensor_log_load_transfer(uint16_t normal_force_fl, uint16_t normal_force_fr, uint16_t normal_force_rl, uint16_t normal_force_rr, uint16_t downforce);

void can_tx_sensor_log_speed_estimation(uint16_t est_speed_fl, uint16_t est_speed_fr, uint16_t est_speed_rl, uint16_t est_speed_rr, float est_speed_vehicle);

void can_tx_sensor_log_traction_control_components(float tc_fb_fl, float tc_fb_fr, float tc_fb_rl, float tc_fb_rr, float tc_ff_fl, float tc_ff_fr, float tc_ff_rl, float tc_ff_rr);

#endif // CAN_SENSOR_TX_GENERATED_H