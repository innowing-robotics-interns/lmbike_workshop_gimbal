/**
 * @file gimbal_hardware.h
 * @brief Public hardware API for the phone gimbal.
 *
 * Hides USART IMU parsing, bit-bang I2C, and PCA9685 pulse generation.
 * Control code should use only these getters/setters.
 *
 * IMU link is USART1 @ 115200 (Yahboom 0x26 Euler frames), not I2C.
 */

#ifndef GIMBAL_HARDWARE_H
#define GIMBAL_HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#define AXIS_ROLL   0
#define AXIS_PITCH  1

void  gimbal_hardware_init(void);
int   gimbal_hardware_update(void);   /* drain UART; 1 = new Euler frame */

float get_roll_angle(void);           /* raw degrees from last good frame */
float get_pitch_angle(void);
float get_dt_seconds(void);           /* since previous new frame; 0 on first */

void  set_servo_pulse(int axis, int pulse_us);  /* clamps inside hardware */

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_HARDWARE_H */
