# Workshop notes: PID on the phone gimbal

Edit **only** [`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c). Drivers live behind [`gimbal_hardware.h`](Core/Inc/gimbal_hardware.h) — leave them alone.

## Layout

| File | Role |
|------|------|
| `gimbal_hardware.*` | IMU UART, PCA9685 I2C, getters/setters |
| `gimbal_control.*` | PID algebra using that API |
| `main.c` | Init + `gimbal_hardware_update()` → `gimbal_control_step()` |

## Hardware API (what control code may call)

```c
float get_roll_angle(void);     /* degrees, raw Euler */
float get_pitch_angle(void);
float get_dt_seconds(void);     /* since previous new IMU frame; 0 on first */
void  set_servo_pulse(int axis, int pulse_us);  /* AXIS_ROLL / AXIS_PITCH */
```

An IMU low-pass (`imu_lpf_update`) exists inside `gimbal_hardware.c` but is **not wired** — getters return raw angles.

## Control law

```text
error = angle_deg
cmd   = sign * (Kp*error + Ki*integral + Kd*derror/dt)
us    = CENTRE + cmd   /* then clamp ±MAX_US, then SERVO_MIN/MAX */
```

Tune `KP` / `KI` / `KD` / `SIGN` arrays in `gimbal_control.c`. Set `Ki=Kd=0` for P-only.

## Suggested demos (gains / hardware only)

1. **P** — `KI=KD=0`, raise `KP`, feel shake.
2. **D** — raise `KD` modestly (~0.15) to damp.
3. **I** — real centre/horn bias (not a software disturbance); enable small `KI`; hold off-axis to feel anti-windup.

## Bridge to the bicycle

Same P/I/D vocabulary; different plant (position servo vs torque/steering). Gains do not transfer.
