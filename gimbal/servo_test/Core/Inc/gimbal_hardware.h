/**
 * @file gimbal_hardware.h
 * @brief Public hardware API for the phone gimbal.
 *
 * Full reference: CONTROL_API.md in the servo_test directory.
 * Control code uses the getters and set_servo_pulse. main calls init and update.
 *
 * IMU link is USART1 @ 115200 (Yahboom 0x26 Euler frames), not I2C.
 */

#ifndef GIMBAL_HARDWARE_H
#define GIMBAL_HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#define AXIS_ROLL   0   /* axis index 0; PCA9685 channel 0 */
#define AXIS_PITCH  1   /* axis index 1; PCA9685 channel 8, not channel 1 */

/* Park both servos at 1500 us, then start the IMU UART.
 * Angles and dt stay 0 until the first accepted frame. */
void  gimbal_hardware_init(void);

/* Drain the IMU UART. Returns 1 when a new Euler frame was accepted, else 0.
 * The first accepted frame reports dt of 0 seconds. */
int   gimbal_hardware_update(void);

/* Roll in degrees, unfiltered. 0.0f until the first good frame. */
float get_roll_angle(void);

/* Pitch in degrees, unfiltered. 0.0f until the first good frame. */
float get_pitch_angle(void);

/* Seconds since the previous accepted frame.
 * 0.0f on the first frame and when two frames share a millisecond. */
float get_dt_seconds(void);

/* axis is AXIS_ROLL or AXIS_PITCH. Any other axis is ignored.
 * pulse_us is clamped here to 500..2500 microseconds. */
void  set_servo_pulse(int axis, int pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_HARDWARE_H */
