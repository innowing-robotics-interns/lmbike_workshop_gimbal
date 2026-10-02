/**
 * @file gimbal_control.h
 * @brief Control-law step: angles in via hardware API, servo pulses out.
 */

#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void gimbal_control_step(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CONTROL_H */
