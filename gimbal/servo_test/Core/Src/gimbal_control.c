/**
 * @file gimbal_control.c
 * @brief Gimbal PID using only the gimbal_hardware.h API.
 *
 * Tune KP/KI/KD/SIGN below. Set Ki=Kd=0 for a pure P law.
 * Do not touch USART, I2C, or the PCA9685 from this file.
 */

#include "gimbal_control.h"
#include "gimbal_hardware.h"

#define CENTRE      1500
#define MAX_US       400
#define SERVO_MIN   1000
#define SERVO_MAX   2000

/* Tune here. Ki=Kd=0 → P-only (same feel as the old gain=3 law). */
static const float KP[2]   = { 3.0f, 3.0f };
static const float KI[2]   = { 0.05f, 0.05f };
static const float KD[2]   = { 0.15f, 0.15f };
static const float SIGN[2] = { 1.0f, 1.0f };

static float integral[2];
static float prev_error[2];
static int   primed[2];

static int axis_pid_us(int axis, float angle_deg)
{
  float error = angle_deg;          /* target level = 0 deg */
  float dt    = get_dt_seconds();
  float dterm = 0.0f;

  if (primed[axis] && dt > 0.0f)
    dterm = (error - prev_error[axis]) / dt;

  float p = KP[axis] * error;
  float i = KI[axis] * integral[axis];
  float d = KD[axis] * dterm;
  float cmd = SIGN[axis] * (p + i + d);

  /* Anti-windup: integrate only if not jammed against ±MAX_US
   * (or error is pulling back toward centre). */
  if (primed[axis] && dt > 0.0f)
  {
    int hi = (cmd >=  (float)MAX_US);
    int lo = (cmd <= -(float)MAX_US);
    if ((!hi && !lo) ||
        (hi && SIGN[axis] * error < 0.0f) ||
        (lo && SIGN[axis] * error > 0.0f))
      integral[axis] += error * dt;
    i = KI[axis] * integral[axis];
    cmd = SIGN[axis] * (p + i + d);
  }

  if (cmd >  (float)MAX_US) cmd =  (float)MAX_US;
  if (cmd < -(float)MAX_US) cmd = -(float)MAX_US;

  prev_error[axis] = error;
  primed[axis] = 1;

  int us = CENTRE + (int)cmd;
  if (us < SERVO_MIN) us = SERVO_MIN;
  if (us > SERVO_MAX) us = SERVO_MAX;
  return us;
}

void gimbal_control_step(void)
{
  set_servo_pulse(AXIS_ROLL,  axis_pid_us(AXIS_ROLL,  get_roll_angle()));
  set_servo_pulse(AXIS_PITCH, axis_pid_us(AXIS_PITCH, get_pitch_angle()));
}
