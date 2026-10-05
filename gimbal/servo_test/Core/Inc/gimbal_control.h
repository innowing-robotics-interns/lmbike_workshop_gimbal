/**
 * @file gimbal_control.h
 * @brief Control-law step: angles in via the hardware API, servo pulses out.
 *
 * Full reference: CONTROL_API.md in the servo_test directory.
 */

#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* One PID update for roll and pitch.
 * Call only when gimbal_hardware_update has just returned non-zero.
 * A second call on the same frame adds error * dt again. */
void gimbal_control_step(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CONTROL_H */
