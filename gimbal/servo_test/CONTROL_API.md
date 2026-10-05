# Control layer API

Reference for the phone-gimbal control code. Each function, macro, and tunable is in its own section.

Edit the control law in [`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c). The drivers stay behind [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h). Include that header from the control file and call only the functions listed here.

```text
main.c
  gimbal_hardware_init()          once, after the LED flashes
  loop:
    gimbal_hardware_update()      1 when a new Euler frame arrived
      gimbal_control_step()       only on that 1
        get_roll_angle()
        get_pitch_angle()
        get_dt_seconds()          inside the per-axis helper
        set_servo_pulse()
```

## How to read a C signature

The bike lessons call Python functions. These declarations are the same idea, written in C. This is the line in the header:

```c
void set_servo_pulse(int axis, int pulse_us);
```

- The word before the name is the return type. `void` means there is nothing to assign. `float` is a decimal number. `int` is a whole number.
- The names in the parentheses are the inputs, in order. You pass values at the call. You do not write the types again at the call.
- A trailing `;` in the header is the declaration. The body lives in the matching `.c` file.
- A `#define` such as `AXIS_ROLL` is a named constant. The compiler substitutes the number. It is not a function call.

A call looks like this:

```c
float roll = get_roll_angle();
set_servo_pulse(AXIS_ROLL, 1500);
```

## Contents

**Lifecycle and control step**

- [`gimbal_hardware_init`](#gimbal_hardware_init)
- [`gimbal_hardware_update`](#gimbal_hardware_update)
- [`gimbal_control_step`](#gimbal_control_step)

**Sensors**

- [`get_roll_angle`](#get_roll_angle)
- [`get_pitch_angle`](#get_pitch_angle)
- [`get_dt_seconds`](#get_dt_seconds)

**Actuator**

- [`AXIS_ROLL`](#axis_roll)
- [`AXIS_PITCH`](#axis_pitch)
- [`set_servo_pulse`](#set_servo_pulse)

**Inside the control file**

- [`axis_pid_us`](#axis_pid_us)
- [`KP`](#kp)
- [`KI`](#ki)
- [`KD`](#kd)
- [`SIGN`](#sign)
- [`CENTRE`](#centre)
- [`MAX_US`](#max_us)
- [`SERVO_MIN`](#servo_min)
- [`SERVO_MAX`](#servo_max)

**Worked step**

- [One control step, in numbers](#one-control-step-in-numbers)

**Out of bounds**

- [Not part of the API](#not-part-of-the-api)

---

## gimbal_hardware_init

### Signature

```c
void gimbal_hardware_init(void);
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h). Body: [`Core/Src/gimbal_hardware.c`](Core/Src/gimbal_hardware.c).

### Job

Bring up the servo driver and the IMU UART, and park both servos at mid-travel.

### Who calls it

`main`, once, after the three LED flashes and before the `while` loop. The control file does not call it.

### Arguments

None.

### Return value

None. The function is `void`.

### Units

The parked pulse is 1500 microseconds on roll and on pitch. The IMU UART runs at 115200 bits per second. The park is held for about 500 milliseconds before the UART starts. That 500 ms wait is inside this function, so `main` does not continue until it finishes.

### When it is valid

Call it once, after `HAL_Init`, the system clock, and GPIO init have run. `main` already does that. A second call is not part of the normal program.

### What it does not do

- It does not read an angle.
- It does not run the PID.
- It does not start a timer callback. Later frames arrive only because `main` keeps calling `gimbal_hardware_update`.

### Edge cases

- After this returns, and before the first accepted IMU frame, `get_roll_angle` and `get_pitch_angle` are `0.0f`, and `get_dt_seconds` is `0.0f`.
- The servos sit at 1500 µs through the 500 ms wait. Nothing is correcting tilt yet.
- Static PID memory in the control file (`integral`, `prev_error`, `primed`) is still zero. This function does not clear it later. Reset comes from a new flash or power cycle.

### Numeric example

After a normal boot the first commands the hardware has sent are:

```text
set point, roll  -> 1500 us
set point, pitch -> 1500 us
```

Those are the park pulses from init, not a PID result.

### A wrong call

Calling `gimbal_control_step` before this function runs writes servo pulses onto a bus that has not been brought up. Leave the call where `main` has it.

---

## gimbal_hardware_update

### Signature

```c
int gimbal_hardware_update(void);
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h). Body: [`Core/Src/gimbal_hardware.c`](Core/Src/gimbal_hardware.c).

### Job

Read waiting IMU bytes. Report whether a new Euler attitude is ready for one control step.

### Who calls it

`main`, at the top of every loop. The control file does not call it.

```c
while (1)
{
  if (gimbal_hardware_update() != 0)
    gimbal_control_step();
}
```

### Arguments

None.

### Return value

| Value | Meaning |
|-------|---------|
| `1` | A new Euler frame was accepted during this call. Angles and `dt` match that update. Run one control step. |
| `0` | No new Euler frame. Angles and `dt` stay as they were. Skip the control step. |

The return type is `int`. Treat any non-zero as "a frame arrived". The body returns only `0` or `1`.

### Units

On a `1` return, `get_dt_seconds` is in seconds. The clock is `HAL_GetTick`, which counts milliseconds, so a non-zero `dt` is a multiple of `0.001` seconds.

### When it is valid

Call it after `gimbal_hardware_init`. Call it often. Most calls return `0`, because the IMU has not finished a new frame yet. That `0` is the normal idle result, not a fault by itself.

### What it does not do

- It does not move a servo.
- It does not run `gimbal_control_step`.
- It does not apply the low-pass that exists inside the driver. Getters stay on the raw degrees.
- It does not expose the third angle packed in the Euler payload. Only roll and pitch are stored.

### Edge cases

- The first accepted frame sets `dt` to `0.0f` on purpose. Derivative and integral both need a previous sample.
- A later frame whose timestamp falls in the same millisecond as the previous accepted frame also stores `dt` of `0.0f`.
- One call reads at most 64 waiting bytes, then returns. Bytes still in the UART wait for the next call. `main` calls this every loop, so the rest is drained on the following passes.
- If several Euler frames sit in those 64 bytes, the stored angles become the **last** frame in that burst. `dt` is the time since the previous call that returned `1`, and `gimbal_control_step` runs once for the burst.
- Other UART sentences are parsed and then discarded. A return of `1` means an Euler frame (function code `0x26`, length 17) passed its checksum.
- A `0` return leaves the previous angles and the previous `dt` unchanged.

### Numeric example

```text
call 1..40   return 0    angles still 0, dt still 0
call 41      return 1    roll = 10 deg, pitch = 0 deg, dt = 0
call 42..50  return 0    those same angles, dt stays 0
call 51      return 1    roll = 6 deg,  dt = 0.020   (20 ms later)
```

The control step belongs on call 41 and call 51 only.

### A wrong call

```c
while (1)
{
  gimbal_hardware_update();
  gimbal_control_step();   /* also runs when update returned 0 */
}
```

On the idle calls, `dt` is still the last frame's interval. The integral adds `error * dt` again for a sample that already counted. Gate the step with `!= 0`.

---

## gimbal_control_step

### Signature

```c
void gimbal_control_step(void);
```

Header: [`Core/Inc/gimbal_control.h`](Core/Inc/gimbal_control.h). Body: [`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c).

### Job

Run one PID update for roll and one for pitch, then write both servo pulses.

### Who calls it

`main`, and only inside the branch where `gimbal_hardware_update` returned non-zero.

### Arguments

None. The step reads the sensors itself:

```c
set_servo_pulse(AXIS_ROLL,  axis_pid_us(AXIS_ROLL,  get_roll_angle()));
set_servo_pulse(AXIS_PITCH, axis_pid_us(AXIS_PITCH, get_pitch_angle()));
```

### Return value

None. The function is `void`. The pulses go out through `set_servo_pulse`. There is no status to check.

### Units

Inputs are degrees and seconds, taken from the getters. Outputs are pulse widths in microseconds.

### When it is valid

Call it once per accepted IMU frame, immediately after `gimbal_hardware_update` returns `1`. Both axes are updated together. Roll and pitch share that frame's `dt`.

### What it does not do

- It does not talk to USART, I2C, or PCA9685 registers. Those calls stay inside `set_servo_pulse` and the init path.
- It does not take a setpoint argument. The target angle is `0` degrees, written as `error = angle_deg` in `axis_pid_us`.
- It does not reset `integral`, `prev_error`, or `primed`. That memory lives for the whole run.

### Edge cases

- Called once on the first frame: derivative is `0`, the integral is left at `0`, and both axes become "primed" for the next frame.
- Called a second time without a new `update` returning `1`: the same `dt` is applied again, so the integral grows a second time for one real interval.
- Pitch and roll each keep their own integral, previous error, and primed flag. A roll sample does not overwrite pitch history.

### Numeric example

With the shipped gains, roll `+10` degrees on the first frame (`dt` is `0`), pitch `0`:

```text
roll  pulse = 1530 us
pitch pulse = 1500 us
```

The full arithmetic is in [One control step, in numbers](#one-control-step-in-numbers).

### A wrong call

```c
void gimbal_control_step(void)
{
  set_servo_pulse(AXIS_ROLL, 1500 + (int)(3.0f * get_roll_angle()));
  /* pitch never written: that servo stays where init parked it */
}
```

The shipped function writes both axes on every call. Dropping one axis leaves that servo at its last pulse.

---

## get_roll_angle

### Signature

```c
float get_roll_angle(void);
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h).

### Job

Return the roll attitude from the last accepted IMU frame.

### Who calls it

`gimbal_control_step`, once per step, as the measurement for `AXIS_ROLL`.

### Arguments

None.

### Return value

Roll in degrees, as a `float`.

### Units

Degrees, converted inside the driver from the IMU's radians (`radians * 57.29578`). Positive and negative depend on which way the board is mounted. A level phone is the reading you want near `0`. This API does not add a mount offset.

### When it is valid

Any time after `gimbal_hardware_init`. The value is meaningful after `gimbal_hardware_update` has returned `1` at least once. Between accepted frames the function keeps returning the last angle. That is the sample the next step should use, and it is also why the step must not run on a `0` return: the angle would be counted again.

### What it does not do

- It does not return radians.
- It does not subtract a target. The number is the measured angle.
- It does not low-pass the sample. `imu_lpf_update` is compiled into the driver and is not called.
- It does not start a new UART read. The read happens in `gimbal_hardware_update`.

### Edge cases

- Before the first good frame the stored roll is `0.0f`. A control step in that window would command the centre pulse and look like a perfect level.
- A payload float that is NaN is stored as `0.0f`.
- There is no yaw getter. The third value in the Euler payload is discarded.

### Numeric example

```c
float roll = get_roll_angle();   /* 10.0f means +10 degrees */
```

Shipped P term for that reading, `KP[0] = 3`:

```text
P = 3.0 * 10.0 = 30 us
```

### A wrong call

```c
float roll_rad = get_roll_angle() * 0.0174533f;  /* treats degrees as if they were still radians */
```

The conversion to degrees has already happened. Multiply by `KP` directly.

---

## get_pitch_angle

### Signature

```c
float get_pitch_angle(void);
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h).

### Job

Return the pitch attitude from the last accepted IMU frame.

### Who calls it

`gimbal_control_step`, once per step, as the measurement for `AXIS_PITCH`.

### Arguments

None.

### Return value

Pitch in degrees, as a `float`.

### Units

Degrees, same conversion as roll (`radians * 57.29578`). Sign follows the mount. Level is the reading near `0`. Roll and pitch are independent samples from the same frame. They do not share a number.

### When it is valid

Same window as `get_roll_angle`: after init, and meaningful once `gimbal_hardware_update` has returned `1`. Until the next `1`, this keeps returning the pitch from that frame.

### What it does not do

- It does not return roll. The two getters are separate stored values.
- It does not return radians, a target error, or a filtered angle.
- It does not read the UART by itself.

### Edge cases

- Before the first good frame the stored pitch is `0.0f`.
- A NaN payload float is stored as `0.0f`.
- A burst of frames inside one `gimbal_hardware_update` keeps only the last pitch.

### Numeric example

```c
float pitch = get_pitch_angle();  /* 0.0f while the nose is level */
```

Shipped step with pitch `0` and `SIGN[1] = 1` produces a pitch command of `0` us away from centre, so the pulse is `1500` us, as long as the integral is still `0` and the derivative is `0`.

### A wrong call

```c
set_servo_pulse(AXIS_PITCH, 1500 + (int)(3.0f * get_roll_angle()));
```

That wires the pitch servo to the roll measurement. Use `get_pitch_angle` for `AXIS_PITCH`.

---

## get_dt_seconds

### Signature

```c
float get_dt_seconds(void);
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h).

### Job

Return the time since the previous accepted IMU frame.

### Who calls it

`axis_pid_us`, once per axis per step. Both axes see the same value on a given step, because `gimbal_hardware_update` stores one interval for the frame.

### Arguments

None.

### Return value

Elapsed time in seconds, as a `float`.

### Units

Seconds. The source counter steps by 1 millisecond, so the smallest non-zero result is `0.001f` (1 ms). A 50 Hz stream is about `0.020f`. The rate is whatever the IMU actually delivers. There is no fixed sample period in this API.

### When it is valid

Read it inside the control step that follows a `1` from `gimbal_hardware_update`. That is the interval for the sample you just took.

The first accepted frame returns `0.0f`. Use the derivative and the integral only when the value is greater than `0` and this axis already has a previous error. `axis_pid_us` does that with `primed[axis] && dt > 0.0f`.

### What it does not do

- It does not return milliseconds. Multiply by `1000` yourself if you want ms.
- It does not measure the time since the previous call of `get_dt_seconds`. Several reads in one step return the same stored number.
- It does not advance when `gimbal_hardware_update` returns `0`.

### Edge cases

- First accepted frame: `0.0f`.
- Two accepted frames in the same millisecond: `0.0f` again.
- Dividing by this value when it is `0` is undefined for the derivative. The shipped helper skips that division. A new line of the form `(error - prev) / get_dt_seconds()` needs the same guard.
- On a `0` return from update, this still reports the old interval. Running the step anyway reapplies that old interval.

### Numeric example

```text
first frame     get_dt_seconds() == 0.0f
20 ms later     get_dt_seconds() == 0.020f
same ms again   get_dt_seconds() == 0.0f
```

Integral contribution of a `6` degree error over that 20 ms, before `KI` is applied:

```text
error * dt = 6 * 0.020 = 0.12 degree-seconds
```

### A wrong call

```c
float dterm = (error - prev_error) / get_dt_seconds();
```

On the first frame `get_dt_seconds()` is `0.0f`. Guard it:

```c
float dterm = 0.0f;
float dt = get_dt_seconds();
if (primed && dt > 0.0f)
  dterm = (error - prev_error) / dt;
```

---

## AXIS_ROLL

### Definition

```c
#define AXIS_ROLL   0
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h).

### Job

Name the roll axis for `set_servo_pulse` and for the gain arrays.

### Who uses it

`gimbal_control_step`, as the axis argument to `axis_pid_us` and to `set_servo_pulse`.

### Value

Integer `0`.

On the PCA9685 this selects channel `0`. In `gimbal_control.c` it is also the index of the roll entry in `KP`, `KI`, `KD`, `SIGN`, `integral`, `prev_error`, and `primed`.

### Units

An axis index, not degrees and not microseconds.

### When it is valid

Pass it as the `axis` argument. Use index `0` when you edit the roll entry of a gain array.

### What it does not do

- It does not read the angle. Pair it with `get_roll_angle`.
- It does not choose the servo horn's direction. That is `SIGN[0]`.
- It is not a pulse width.

### Edge cases

The preprocessor replaces the name with `0` before compile. A debugger watch on the macro itself has nothing to show. Watch the pulse or the angle instead.

### Numeric example

```c
set_servo_pulse(AXIS_ROLL, 1530);   /* channel 0, 1530 us */
```

```c
static const float KP[2] = { 3.0f, 3.0f };  /* KP[AXIS_ROLL] is the first number */
```

### A wrong use

```c
set_servo_pulse(AXIS_ROLL, 0);    /* pulse 0 us, then hardware raises it to the 500 us backstop */
float roll = AXIS_ROLL;           /* roll becomes 0, which is the index, not the angle */
```

---

## AXIS_PITCH

### Definition

```c
#define AXIS_PITCH  1
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h).

### Job

Name the pitch axis for `set_servo_pulse` and for the gain arrays.

### Who uses it

`gimbal_control_step`, as the axis argument for the pitch half of the step.

### Value

Integer `1`.

On the PCA9685 this selects channel `8`. The header number and the array index are different on purpose: the array index is `1`, the servo header is channel `8`. In `gimbal_control.c`, `1` is the pitch entry of `KP`, `KI`, `KD`, `SIGN`, `integral`, `prev_error`, and `primed`.

### Units

An axis index, not degrees and not microseconds.

### When it is valid

Pass it as the `axis` argument. Use index `1` when you edit the pitch entry of a gain array.

### What it does not do

- It does not read the angle. Pair it with `get_pitch_angle`.
- It does not move channel `1` of the PCA9685. Channel `1` is unused. Pitch is channel `8`.
- It is not a pulse width.

### Edge cases

Plugging the pitch servo into PCA channel `1` because the macro is `1` leaves `set_servo_pulse(AXIS_PITCH, ...)` talking to channel `8`. The two headers are independent. The macro is the only map.

### Numeric example

```c
set_servo_pulse(AXIS_PITCH, 1500);  /* channel 8, 1500 us */
```

```c
static const float KI[2] = { 0.05f, 0.05f };  /* KI[AXIS_PITCH] is the second number */
```

### A wrong use

```c
set_servo_pulse(8, 1500);   /* 8 is the PCA channel, not a legal axis index; the call is ignored */
```

---

## set_servo_pulse

### Signature

```c
void set_servo_pulse(int axis, int pulse_us);
```

Header: [`Core/Inc/gimbal_hardware.h`](Core/Inc/gimbal_hardware.h).

### Job

Command one servo to a pulse width in microseconds.

### Who calls it

`gimbal_control_step`, twice per step: once with `AXIS_ROLL`, once with `AXIS_PITCH`. `gimbal_hardware_init` parks both servos through an internal helper at `1500` us. It does not go through this function, but the pulse you observe at boot is that same 1500 us centre.

### Arguments

| Argument | Type | Meaning |
|----------|------|---------|
| `axis` | `int` | `AXIS_ROLL` (`0`) or `AXIS_PITCH` (`1`). |
| `pulse_us` | `int` | Pulse width in microseconds. Whole microseconds. A fractional command has to be converted before the call. The shipped code uses `(int)cmd`, which truncates toward zero. |

### Return value

None. The function is `void`. An ignored axis produces no return code. Watch the mechanism, or compare the pulse you intended with the limits below.

### Units

Microseconds of servo pulse. Mid-scale for this gimbal is `1500`. A larger number and a smaller number move the horn in opposite directions. Which physical direction that is depends on the horn and on `SIGN`.

### When it is valid

After `gimbal_hardware_init`. From the control file, call it from `gimbal_control_step` so each accepted IMU frame produces one pulse per axis.

### What it does not do

- It does not clamp to `1000..2000`, and it does not apply `MAX_US`. Those limits are in `axis_pid_us`, before this call.
- It does not know about degrees, gains, or `dt`.
- It does not move both servos. One call, one axis.
- A repeated pulse that maps to the same PCA count as the last accepted write is not sent again.

### Edge cases

- Hardware backstop: values below `500` are raised to `500`. Values above `2500` are lowered to `2500`. `500` and `2500` are the limits inside this function.
- The control file then aims at a tighter window. Shipped `axis_pid_us` only asks for `1100..1900`, because `CENTRE` is `1500` and `MAX_US` is `400`, and it also clamps to `SERVO_MIN` (`1000`) and `SERVO_MAX` (`2000`). With the shipped constants the `1000..2000` clamp never moves the number. It starts to matter if `CENTRE` or `MAX_US` is edited so that `CENTRE + cmd` falls outside `1000..2000`.
- `axis` other than `0` or `1`: the function returns without writing. That includes `8`, which is the pitch PCA channel rather than the axis index.
- Negative `pulse_us` hits the `500` backstop.

### Numeric example

```c
set_servo_pulse(AXIS_ROLL, 1530);    /* sent, channel 0 */
set_servo_pulse(AXIS_ROLL, 1530);    /* same count as last write: not sent again */
set_servo_pulse(AXIS_PITCH, 1500);   /* sent, channel 8 */
set_servo_pulse(AXIS_ROLL, 3000);    /* stored as 2500 us */
set_servo_pulse(AXIS_ROLL, 100);     /* stored as 500 us */
```

Truncation the control file applies before the call:

```text
cmd = -11.994 us
(int)cmd = -11
pulse = 1500 + (-11) = 1489 us
```

### A wrong call

```c
set_servo_pulse(AXIS_ROLL, 1500 + 3 * get_roll_angle());
```

`get_roll_angle` returns `float`. Multiplying by the integer `3` promotes the math to `float`, and the parameter then converts to `int`. Write the gain as `3.0f` and cast in the open, the way `axis_pid_us` does, so the truncation is visible:

```c
set_servo_pulse(AXIS_ROLL, 1500 + (int)(3.0f * get_roll_angle()));
```

---

## axis_pid_us

### Signature

```c
static int axis_pid_us(int axis, float angle_deg);
```

This function is `static` in [`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c). It is not in a header. `main` cannot call it. The description is here because the control step is this function twice.

### Job

Turn one axis's angle into one pulse width, and update that axis's PID memory.

### Who calls it

`gimbal_control_step` only.

### Arguments

| Argument | Type | Meaning |
|----------|------|---------|
| `axis` | `int` | `AXIS_ROLL` or `AXIS_PITCH`. Also the index into `KP`, `KI`, `KD`, `SIGN`, and the history arrays. |
| `angle_deg` | `float` | Measured angle in degrees for that axis. The target is `0`, so the error used below **is** this angle. |

### Return value

`int` pulse width in microseconds, already clamped by `MAX_US` around `CENTRE` and by `SERVO_MIN` / `SERVO_MAX`.

### Units

`angle_deg` is degrees. The return value is microseconds. `get_dt_seconds` supplies seconds for the I and D terms.

### When it is valid

Call it with `0` or `1` only, from the control step, once per axis per accepted frame. The arrays are length 2. Another index reads off the end of them.

### What it does not do

- It does not write the hardware. The caller passes the return value to `set_servo_pulse`.
- It does not take a setpoint. Level is `0` degrees.
- It does not share history between axes.

### Edge cases

Shipped order inside one call:

1. `error = angle_deg`.
2. Derivative is `(error - prev_error) / dt` only when this axis is already primed and `dt > 0`. Otherwise the derivative is `0`.
3. `P = KP * error`, `I = KI * integral` (the integral **before** this sample is added), `D = KD * dterm`, then `cmd = SIGN * (P + I + D)`.
4. If this axis is primed and `dt > 0`, the integral may grow by `error * dt`. The command is then rebuilt with the new integral. See the anti-windup rule below.
5. `cmd` is clamped to the range `-MAX_US .. +MAX_US`.
6. `prev_error` becomes this error, and the axis is marked primed, including on the first sample and including when `dt` was `0`.
7. `us = CENTRE + (int)cmd`, then clamped to `SERVO_MIN .. SERVO_MAX`.

Anti-windup, in the condition the file uses. Let `signed_error = SIGN * error`.

- Inside the rail (`-MAX_US < cmd < MAX_US` before the final clamp): add `error * dt`.
- Command already at or beyond `+MAX_US`, and `signed_error` is negative: still add `error * dt`. The measurement has crossed to the other side of level, so the integral is allowed to unwind.
- Command already at or beyond `-MAX_US`, and `signed_error` is positive: still add `error * dt`, for the same reason.
- Command on a rail and `signed_error` still points the same way as that rail: leave the integral unchanged.

The rail test uses the command from step 3, before the integral grows.

`(int)cmd` truncates toward zero. `15.9` becomes `15`. `-11.994` becomes `-11`.

### Numeric example

Roll, shipped gains, second frame: angle `+6` degrees, previous error `+10` degrees, `dt = 0.02` s, integral still `0`.

```text
dterm = (6 - 10) / 0.02 = -200 deg/s
P = 3 * 6 = 18
I = 0.05 * 0 = 0
D = 0.15 * -200 = -30
cmd before integrate = 18 + 0 - 30 = -12
integral becomes 0 + 6 * 0.02 = 0.12
I becomes 0.05 * 0.12 = 0.006
cmd = 18 + 0.006 - 30 = -11.994
pulse = 1500 + (int)(-11.994) = 1489 us
```

### A wrong call

```c
us = axis_pid_us(AXIS_ROLL, 0.0f - get_roll_angle());
```

The helper already treats the angle as the error, because the target is `0`. Subtracting the angle from `0` flips the sign a second time, on top of `SIGN`.

The bicycle lessons formed error as target minus measurement. Here the target is `0`, and the file stores `error = angle_deg`. `SIGN` is the place that chooses direction.

---

## KP

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
static const float KP[2] = { 3.0f, 3.0f };
```

Index `0` is roll (`AXIS_ROLL`). Index `1` is pitch (`AXIS_PITCH`).

### Job

Proportional gain. Scales the current angle into microseconds.

### Default

`3.0` us per degree, both axes.

### Effect of changing it

```text
P = KP[axis] * error
```

`error` is the angle in degrees. `KP = 0` removes the P term. A larger magnitude pushes harder for the same tilt. The sign of the physical motion is `SIGN`, so leave `KP` positive and flip `SIGN` to reverse an axis.

`KI` and `KD` at `0` leave a pure P law: `pulse = CENTRE + (int)(SIGN * KP * angle)`.

### What it does not do

It does not set the pulse by itself. I, D, `SIGN`, and the clamps still run.

### Numeric example

Angle `+10` degrees, `KP = 3`, integral `0`, derivative `0`, `SIGN = 1`:

```text
cmd = 30 us
pulse = 1530 us
```

### A wrong edit

```c
static const float KP[2] = { -3.0f, 3.0f };
```

A negative `KP` reverses roll, which is what `SIGN[0] = -1` is for. Mixing both makes the axis hard to reason about. Change `SIGN` to reverse an axis.

---

## KI

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
static const float KI[2] = { 0.05f, 0.05f };
```

Index `0` is roll. Index `1` is pitch.

### Job

Integral gain. Scales the accumulated angle-times-time into microseconds, so a small steady tilt can still grow a command.

### Default

`0.05` us per (degree-second), both axes.

### Effect of changing it

```text
integral += error * dt          /* when the anti-windup test allows it */
I = KI[axis] * integral
```

`KI = 0` removes the I term. The stored `integral` may still change, and the command contribution stays `0`.

The added piece uses the angle in degrees and `dt` in seconds, so the unit of `integral` is degree-seconds. The first frame does not add, because that axis is not primed yet and `dt` is `0`.

A steady hold off level (horn or centre bias, not a line you add in software) is the situation I is for. A large `KI` winds into the `MAX_US` rail and the phone feels like it is fighting a sticky offset.

### What it does not do

- It does not create the steady bias. The bias is mechanical. `KI` is the response to it.
- It does not bypass anti-windup. On the rail, growth in the saturating direction stops. See [`axis_pid_us`](#axis_pid_us).

### Numeric example

One allowed update, error `+6` degrees, `dt = 0.02` s, integral was `0`, `KI = 0.05`:

```text
integral = 0.12 degree-seconds
I = 0.006 us
```

That `0.006` us is smaller than the `(int)` truncation until the integral has been adding for many frames.

### A wrong edit

Setting `KI` to a large value while holding the phone against the stop, and reading the creep toward `MAX_US` as "the angle gain". That creep is the integral. Put `KI` back to `0` to see P and D alone.

---

## KD

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
static const float KD[2] = { 0.15f, 0.15f };
```

Index `0` is roll. Index `1` is pitch.

### Job

Derivative gain. Scales the rate of change of the angle into microseconds, which damps a fast swing.

### Default

`0.15` us per (degree/second), both axes.

### Effect of changing it

```text
dterm = (error - prev_error) / dt     /* primed and dt > 0 */
D = KD[axis] * dterm
```

`KD = 0` removes the D term. The rate is in degrees per second. A modest `0.15` is the shipped damping. Raising it further opposes quick motion more, and high enough it buzzes, because IMU noise becomes a large `dterm` when `dt` is a few milliseconds.

The first frame forces `D` to `0` even if `KD` is non-zero.

### What it does not do

- It does not filter the angle. The getter is raw, so D sees that raw step.
- It does not divide by `dt` when `dt` is `0`. The helper leaves `dterm` at `0` in that case.

### Numeric example

Error moves from `+10` degrees to `+6` degrees in `0.02` s, `KD = 0.15`:

```text
dterm = -200 deg/s
D = -30 us
```

`D` is `-30` µs. The angle is decreasing, so this term opposes that motion. It subtracts from the positive P term instead of adding another push toward level.

### A wrong edit

```c
float dterm = (error - prev_error) / get_dt_seconds();
```

added beside the helper, without the `dt > 0` test. Keep the guard that is already in `axis_pid_us`.

---

## SIGN

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
static const float SIGN[2] = { 1.0f, 1.0f };
```

Index `0` is roll. Index `1` is pitch.

### Job

Direction of the command for that axis. `+1` keeps `P + I + D` as computed. `-1` reverses that axis.

### Default

`+1` on roll and `+1` on pitch. These match the mount the project was tuned on. A rebuilt horn or a rotated IMU can require one of them to be `-1`.

### Effect of changing it

```text
cmd = SIGN[axis] * (P + I + D)
```

`SIGN` also enters the anti-windup test as `SIGN * error`. Flip one entry at a time.

The correct sign opposes tilt: a positive angle produces a pulse that rotates the phone back toward `0`. The wrong sign increases the tilt. The horn runs to the pulse rail and stays there.

### What it does not do

It does not change which PCA channel moves. Channel choice is `AXIS_ROLL` / `AXIS_PITCH`. A runaway with the correct sign is a gain or a mechanical problem. A runaway that swaps direction when `SIGN` flips was a direction problem.

### Numeric example

`P + I + D = 30` us.

```text
SIGN =  1  ->  cmd =  30  ->  pulse = 1530 us
SIGN = -1  ->  cmd = -30  ->  pulse = 1470 us
```

### A wrong edit

```c
static const float SIGN[2] = { -1.0f, -1.0f };
```

as the first experiment. Change a single axis, watch that axis, then put it back before editing the other one.

---

## CENTRE

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
#define CENTRE  1500
```

### Job

Pulse width, in microseconds, added after the signed command. It is the pulse at zero command.

### Default

`1500` microseconds.

### Effect of changing it

```text
us = CENTRE + (int)cmd
```

Init parks the servos at `1500` as well, using the hardware file's own centre `(1000 + 2000) / 2`. If you change `CENTRE` here and leave init alone, the servos sit at `1500` during the 500 ms park, then jump to the new centre on the first control step.

`CENTRE + MAX_US` and `CENTRE - MAX_US` are the authority limits before `SERVO_MIN` / `SERVO_MAX`. Shipped: `1900` and `1100`.

### What it does not do

It does not change the angle target. The target stays `0` degrees. A crooked mount is handled by `KI` and by the mechanical centre, not by hiding an angle offset inside `CENTRE`. `CENTRE` is a pulse-width zero, not a degree zero.

### Numeric example

```text
cmd = 0     -> pulse = 1500 us
cmd = 30    -> pulse = 1530 us
cmd = -11   -> pulse = 1489 us
```

### A wrong edit

Putting `CENTRE` at `2000` so a tilted idle "looks level". The PID still regulates angle toward `0`, and the pulse zero has moved, so the horn and the IMU disagree about where level is.

---

## MAX_US

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
#define MAX_US  400
```

### Job

Largest absolute command, in microseconds, away from `CENTRE`. This is the authority limit and the anti-windup rail.

### Default

`400` microseconds. Shipped pulses that come out of the PID therefore stay in `1100..1900` before the `SERVO_MIN` / `SERVO_MAX` check.

### Effect of changing it

After I and D are updated, `cmd` is forced into `-MAX_US .. +MAX_US`. The anti-windup test treats `cmd >= MAX_US` and `cmd <= -MAX_US` as the rails.

A smaller value protects the mechanism and makes I wind up sooner. A larger value allows more horn travel. If `CENTRE + MAX_US` exceeds `2000`, or `CENTRE - MAX_US` is below `1000`, `SERVO_MAX` or `SERVO_MIN` clips the pulse again.

### What it does not do

It is not the hardware backstop. `set_servo_pulse` still accepts `500..2500`. `MAX_US` never asks for that full range unless you change it.

### Numeric example

Command before the clamp is `480` us, `MAX_US` is `400`:

```text
cmd becomes 400
pulse = 1500 + 400 = 1900 us
```

The integral is not increased on that sample when the error is still positive and `SIGN` is `+1`, because the pre-clamp command was already past the rail. Details are under [`axis_pid_us`](#axis_pid_us).

### A wrong edit

Raising `MAX_US` to `2000` and also expecting `SERVO_MAX` to stay out of the way. `CENTRE + 2000` is `3500`, `SERVO_MAX` cuts that to `2000`, and the hardware backstop would have allowed `2500`. Two different clamps are in the path. Know which line changed the number.

---

## SERVO_MIN

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
#define SERVO_MIN  1000
```

### Job

Lowest pulse, in microseconds, that `axis_pid_us` is allowed to return.

### Default

`1000` microseconds.

### Effect of changing it

Applied after `CENTRE + (int)cmd`:

```c
if (us < SERVO_MIN) us = SERVO_MIN;
```

With shipped `CENTRE` and `MAX_US`, the smallest PID pulse is `1100`, so this line does not change the shipped numbers. It is the last guard if the centre or the authority is edited.

### What it does not do

It does not change the clamp inside `set_servo_pulse`. That function's floor is `500` us. A pulse returned from `axis_pid_us` has already been raised to `SERVO_MIN`. A direct `set_servo_pulse` call can still ask for a value between `500` and `999`.

### Numeric example

Someone sets `CENTRE` to `1500` and `MAX_US` to `800`. Command `-800` would be pulse `700`. `SERVO_MIN` raises the return value to `1000`.

### A wrong edit

Lowering `SERVO_MIN` below `500` and expecting the servo to see it. `set_servo_pulse` raises anything below `500` back to `500`.

---

## SERVO_MAX

### Where to edit

[`Core/Src/gimbal_control.c`](Core/Src/gimbal_control.c):

```c
#define SERVO_MAX  2000
```

### Job

Highest pulse, in microseconds, that `axis_pid_us` is allowed to return.

### Default

`2000` microseconds.

### Effect of changing it

```c
if (us > SERVO_MAX) us = SERVO_MAX;
```

Shipped PID pulses top out at `1900`, so this line does not change the shipped numbers. It clips when `CENTRE + MAX_US` is above `2000`.

### What it does not do

It does not change the hardware ceiling. `set_servo_pulse` lowers anything above `2500` to `2500`. Between `2001` and `2500`, only this control-file clamp (or `MAX_US`) keeps the pulse down. A direct call can still send `2200`.

### Numeric example

`CENTRE + MAX_US` edited so the sum is `2300`. `axis_pid_us` returns `2000`. The hardware would have accepted `2300`.

### A wrong edit

Raising `SERVO_MAX` above `2500` and treating that as the pulse the servo receives. The setter stops at `2500`.

---

## One control step, in numbers

Shipped constants: `KP = 3`, `KI = 0.05`, `KD = 0.15`, `SIGN = 1`, `CENTRE = 1500`, `MAX_US = 400`, `SERVO_MIN = 1000`, `SERVO_MAX = 2000`. Pitch is `0` the whole time, integral starts at `0`, both axes start unprimed. Only roll is interesting.

### Frame 1 — first accepted frame

`gimbal_hardware_update` returns `1`. Roll is `+10` degrees. `get_dt_seconds` is `0`. `primed` is `0`.

```text
error = 10
dterm = 0                         /* not primed */
P = 30,  I = 0,  D = 0
cmd = 30                          /* no integral update */
pulse = 1500 + 30 = 1530 us
prev_error = 10,  primed = 1,  integral stays 0
```

Pitch: error `0`, pulse `1500` us, pitch primed flag becomes `1`.

### Frame 2 — 20 ms later, moving toward level

Roll is `+6` degrees. `dt` is `0.02` seconds. Previous error is `10`. Integral is `0`.

```text
dterm = (6 - 10) / 0.02 = -200 deg/s
P = 18
I = 0
D = 0.15 * -200 = -30
cmd = -12                         /* inside the rail, so integrate */
integral = 0 + 6 * 0.02 = 0.12
I = 0.05 * 0.12 = 0.006
cmd = 18 + 0.006 - 30 = -11.994
(int)cmd = -11                    /* toward zero */
pulse = 1500 - 11 = 1489 us
```

### Frame 3 — a fast shove into the rail

Continuing from frame 2: integral `0.12`, previous error `6`, primed. Roll is now `+50` degrees. `dt` is `0.02` seconds.

```text
dterm = (50 - 6) / 0.02 = 2200 deg/s
P = 150
I = 0.05 * 0.12 = 0.006
D = 0.15 * 2200 = 330
cmd = 480.006                     /* already >= MAX_US */
signed error = +50                /* same direction as the high rail */
integral stays 0.12
cmd clamped to 400
pulse = 1900 us
```

`SERVO_MAX` (`2000`) does not move `1900`. The hardware backstop (`2500`) is not involved.

### Unwind, in one line

If a later sample is past the high rail because the integral is large, and the angle has crossed through level so `SIGN * error` is negative, the file **does** add `error * dt`. That addition is negative, so the stored integral shrinks. The symmetric case is the low rail with a positive `SIGN * error`.

---

## Not part of the API

These names exist in [`Core/Src/gimbal_hardware.c`](Core/Src/gimbal_hardware.c). Control code does not call them. They are not declared in `gimbal_hardware.h`.

| Name | Why it is not yours to call |
|------|-----------------------------|
| `i2c_*` | Bit-bang I2C used by the PCA9685 driver. |
| `pca_*` | PCA9685 register writes. `set_servo_pulse` is the supported way to command a servo. |
| `imu_*` | UART setup and the Euler parser. `gimbal_hardware_update` and the getters are the supported way to read attitude. |
| `imu_lpf_update` | A low-pass is compiled in and not called. `get_roll_angle` and `get_pitch_angle` return raw degrees. |

`main` already calls `gimbal_hardware_init`, `gimbal_hardware_update`, and `gimbal_control_step` in the right order. Tuning and small control-law edits stay in `gimbal_control.c`.
