# Control layer API

Documentation for the top-level control layer: what [`gimbal_control`](Core/Src/gimbal_control.c) exposes, and the hardware API it is allowed to call.

Edit the control law only in [`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c). Do not open `gimbal_hardware.c` for tuning.

```text
main.c
  └─ gimbal_hardware_update()     // new IMU frame?
        └─ gimbal_control_step()  // control layer
              ├─ get_roll_angle / get_pitch_angle / get_dt_seconds
              └─ set_servo_pulse(axis, us)
```

---

## Control layer entry point

**Header:** [`Core/Inc/gimbal_control.h`](Core/Inc/gimbal_control.h)  
**Source:** [`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c)

### `void gimbal_control_step(void)`

Run one control update for both axes.

| | |
|---|---|
| **When** | Call only after `gimbal_hardware_update()` returns non-zero (a new Euler frame arrived). |
| **Does** | Reads roll/pitch and `dt`, computes PID commands, writes servo pulses. |
| **Must not** | Touch USART, I2C, PCA9685 registers, or HAL timers directly. |

`main` wiring:

```c
gimbal_hardware_init();
for (;;) {
  if (gimbal_hardware_update())
    gimbal_control_step();
}
```

---

## Hardware API used by the control layer

**Header:** [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h)

Include this from `gimbal_control.c`. These are the only hardware calls the control layer should make.

### Axis IDs

| Macro | Value | Meaning |
|-------|-------|---------|
| `AXIS_ROLL` | `0` | Left–right tilt → PCA9685 channel 0 |
| `AXIS_PITCH` | `1` | Up–down tilt → PCA9685 channel 8 |

### Angle getters

#### `float get_roll_angle(void)`
#### `float get_pitch_angle(void)`

| | |
|---|---|
| **Returns** | Attitude in **degrees** from the last good IMU Euler frame. |
| **Target** | Level is **0°** (gravity). Positive/negative depend on mount orientation. |
| **Notes** | Values are **raw** (no low-pass applied). An LPF exists inside hardware but is not wired. |

#### `float get_dt_seconds(void)`

| | |
|---|---|
| **Returns** | Seconds since the previous new IMU frame that `gimbal_hardware_update()` reported. |
| **First frame** | `0.0f` — skip derivative / integral on the first sample. |
| **Use** | `integral += error * dt`, `derror = (error - prev) / dt` when `dt > 0`. |

### Actuator setter

#### `void set_servo_pulse(int axis, int pulse_us)`

| Argument | Meaning |
|----------|---------|
| `axis` | `AXIS_ROLL` or `AXIS_PITCH` |
| `pulse_us` | Commanded pulse width in microseconds |

| | |
|---|---|
| **Typical mid** | `1500` µs |
| **Useful travel** | about `1000`…`2000` µs |
| **Safety** | Hardware clamps to a wider backstop (~`500`…`2500`) before writing the PCA9685. Still clamp in control code to your authority limit. |
| **Invalid axis** | Ignored (no write). |

### Lifecycle (called from `main`, not from control)

| Function | Role |
|----------|------|
| `void gimbal_hardware_init(void)` | I2C/PCA/IMU bring-up; parks servos at centre. |
| `int gimbal_hardware_update(void)` | Drain UART; return `1` if a new Euler frame is ready, else `0`. |

---

## Control-layer tunables (in `gimbal_control.c`)

Not part of a public header; change these constants in the `.c` file.

| Symbol | Default | Meaning |
|--------|---------|---------|
| `KP[2]` | `3.0, 3.0` | Proportional gain (µs per degree), roll then pitch |
| `KI[2]` | `0.05, 0.05` | Integral gain (µs per degree·s). `0` → no I |
| `KD[2]` | `0.15, 0.15` | Derivative gain (µs per degree/s). `0` → no D |
| `SIGN[2]` | `1.0, 1.0` | Direction. Wrong sign → positive feedback / runaway |
| `CENTRE` | `1500` | Pulse at level (zero error) |
| `MAX_US` | `400` | Max \|command\| away from centre (authority + anti-windup rail) |
| `SERVO_MIN` / `SERVO_MAX` | `1000` / `2000` | Final pulse clamps |

Shipped law (per axis):

```text
error = angle_deg                    // target = 0
cmd   = SIGN * (KP*error + KI*integral + KD*derror/dt)
us    = CENTRE + cmd                 // then ±MAX_US, then SERVO_MIN/MAX
set_servo_pulse(axis, us)
```

---

## Minimal control-layer example

```c
#include "gimbal_control.h"
#include "gimbal_hardware.h"

void gimbal_control_step(void)
{
  float roll  = get_roll_angle();
  float pitch = get_pitch_angle();
  float dt    = get_dt_seconds();   /* use for I/D; 0 on first frame */

  (void)dt; /* replace with your PID */

  /* Example: P-only, Kp = 3, both axes */
  set_servo_pulse(AXIS_ROLL,  1500 + (int)(1.0f * 3.0f * roll));
  set_servo_pulse(AXIS_PITCH, 1500 + (int)(1.0f * 3.0f * pitch));
}
```

The repository ships a full PID in `gimbal_control.c` instead of this sketch.

---

## Rules of thumb

1. Call `gimbal_control_step()` only on a fresh frame (`update() != 0`).
2. Use only the getters/setters above from the control layer.
3. Treat level as `0°`; do not hard-code a setpoint offset into firmware for a crooked mount.
4. Flip `SIGN` one axis at a time if the platform runs away.
5. Set `KI`/`KD` to `0` to feel P or PD alone without changing structure.
