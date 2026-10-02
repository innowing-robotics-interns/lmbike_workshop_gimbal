/* ===========================================================================
 *  minimal_gimbal.c  -  map of the gimbal path (NOT built by CubeIDE)
 * ===========================================================================
 *
 *  Live firmware is split:
 *
 *    Core/Src/gimbal_hardware.c  — IMU UART, I2C/PCA9685, public API
 *    Core/Src/gimbal_control.c   — PID using get_* / set_servo_pulse only
 *    Core/Src/main.c             — init + update() -> gimbal_control_step()
 *
 *  Public API (gimbal_hardware.h):
 *    get_roll_angle / get_pitch_angle / get_dt_seconds / set_servo_pulse
 *    gimbal_hardware_init / gimbal_hardware_update
 *
 *  Data flow:
 *
 *    IMU 115200 --> PA10 --> hardware parser --> get_*_angle()
 *                                                   |
 *                                                   v
 *                                         gimbal_control_step()  PID
 *                                                   |
 *                                                   v
 *                                         set_servo_pulse() --> PCA9685
 *
 *  imu_lpf_update() exists in hardware with EMA math but is NOT called;
 *  getters return raw Euler degrees.
 *
 *  Below is a monolithic sketch of the old single-file path for reading.
 *  Prefer editing gimbal_control.c for the live control law.
 */

/* ---- 1. Hardware and constants ---------------------------------------- */

/*  Pins
 *    PB6  SCL  -> PCA9685 SCL    open-drain; needs 4.7k pull-ups
 *    PB7  SDA  -> PCA9685 SDA    open-drain; needs 4.7k pull-ups
 *    PA10 RX   <- IMU TX         115200 8N1, input with pull-up
 *    PC13      LED (active low)  unused in this file
 */
#define I2C_SCL_HIGH()  (GPIOB->BSRR = (1u << 6))
#define I2C_SCL_LOW()   (GPIOB->BRR  = (1u << 6))
#define I2C_SDA_HIGH()  (GPIOB->BSRR = (1u << 7))
#define I2C_SDA_LOW()   (GPIOB->BRR  = (1u << 7))
#define I2C_SDA_READ()  ((GPIOB->IDR & (1u << 7)) != 0u)

/*  PCA9685, 7-bit address 0x40 (all address jumpers open) */
#define PCA_ADDR          0x40u
#define PCA_MODE1         0x00u
#define PCA_MODE2         0x01u
#define PCA_LED0_ON_L     0x06u   /* channel 0 ON_L; each next channel is +4 */
#define PCA_PRESCALE      0xFEu
#define PCA_PRESCALE_50HZ 121u    /* 25 MHz / (4096 * 50 Hz) - 1 */

/*  IMU (Yahboom); function code 0x26 is the Euler-angle frame */
#define IMU_BAUD        115200u
#define IMU_HDR1        0x7Eu
#define IMU_HDR2        0x23u
#define IMU_FUNC_EULER  0x26u
#define IMU_EULER_LEN   17u       /* 2 hdr + len + func + 12 data + checksum */
#define IMU_MAX_LEN     64u
#define IMU_RAD_TO_DEG  57.29578f

/*  Servos: treat 1000..2000 us as 0..180 degrees. That is a firmware
 *  assumption; it has not been verified with a protractor (the horns may
 *  be the 270-degree variant). */
#define SERVO_MIN_US  1000
#define SERVO_MAX_US  2000

/*  Channel map. Change this line to re-pin a servo — nothing else can detect
 *  a servo plugged into the wrong header. The two headers are independent. */
typedef enum { AXIS_ROLL = 0, AXIS_PITCH = 1, AXIS_COUNT = 2 } axis_t;
static const uint8_t pca_ch[AXIS_COUNT] = { 0u, 8u };

/*  Per-axis PID memory. I and D need history between frames; sharing one
 *  block would let roll overwrite pitch's previous error and integral. */
typedef struct {
  float    integral;
  float    prev_error;
  uint32_t prev_ms;
  uint8_t  primed;
} pid_axis_t;

/*  Control-law parameters. One set per axis: Kp/Ki/Kd, sign, and max travel
 *  are each a hardware/tuning fact, not a shared constant.
 *
 *  With Ki=Kd=0 the law reduces to the old P-only form
 *  us = CENTRE + sign * Kp * angle. */
#define IMU_SERVO_CENTRE      ((SERVO_MIN_US + SERVO_MAX_US) / 2)   /* 1500 */
#define IMU_SERVO_ROLL_KP      3.0f
#define IMU_SERVO_ROLL_KI      0.05f
#define IMU_SERVO_ROLL_KD      0.15f
#define IMU_SERVO_ROLL_SIGN   (1.0f)
#define IMU_SERVO_ROLL_MAX_US  400
#define IMU_SERVO_PITCH_KP     3.0f
#define IMU_SERVO_PITCH_KI     0.05f
#define IMU_SERVO_PITCH_KD     0.15f
#define IMU_SERVO_PITCH_SIGN  (1.0f)
#define IMU_SERVO_PITCH_MAX_US 400

/*  Shared attitude. Written by the parser, read by the control law. */
static volatile float imu_roll_deg;
static volatile float imu_pitch_deg;

/*  PID state, one block per axis. */
static pid_axis_t pid_roll;
static pid_axis_t pid_pitch;

/*  Last counts the chip ACK'd, per axis. Used to skip redundant I2C writes. */
static uint16_t pca_last_counts[AXIS_COUNT] = { 0xFFFFu, 0xFFFFu };


/* ---- 2. Bit-bang I2C -------------------------------------------------- */

/*  Why not hardware I2C1: on this package the I2C1 pins conflict with other
 *  uses, so PB6/PB7 are driven in software. Clock is ~50 kHz, well below the
 *  chip's rated speed. */
static void i2c_delay(void)
{
  for (volatile int i = 0; i < 60; i++) { }   /* ~8 us at 72 MHz */
}

static void i2c_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  (void)RCC->APB2ENR;          /* read-back: wait for the clock before GPIO */

  /*  Configure PB6/PB7 as open-drain outputs (CNF=01, MODE=11 -> 0x7).
   *  Open-drain is required, not a style choice: I2C is a wired-AND bus;
   *  push-pull would fight any device that pulls the line low. */
  GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0x77u << 24);

  I2C_SCL_HIGH();              /* idle: release both lines */
  I2C_SDA_HIGH();
}

static void i2c_start(void)
{
  I2C_SDA_HIGH();
  I2C_SCL_HIGH();
  i2c_delay();
  I2C_SDA_LOW();               /* SDA falling while SCL high = START */
  i2c_delay();
  I2C_SCL_LOW();
}

static void i2c_stop(void)
{
  I2C_SDA_LOW();
  I2C_SCL_HIGH();
  i2c_delay();
  I2C_SDA_HIGH();              /* SDA rising while SCL high = STOP */
  i2c_delay();
}

/*  Send one byte MSB-first, then sample ACK. Returns 1 if the slave ACK'd.
 *  A NACK on the address byte is the most useful bus diagnostic: nobody is
 *  listening. */
static uint8_t i2c_byte(uint8_t b)
{
  uint8_t acked;

  for (int bit = 7; bit >= 0; bit--)
  {
    if (((b >> bit) & 1u) != 0u) I2C_SDA_HIGH();
    else                         I2C_SDA_LOW();
    i2c_delay();
    I2C_SCL_HIGH();
    i2c_delay();
    I2C_SCL_LOW();
  }

  I2C_SDA_HIGH();              /* release SDA so the slave can pull it low */
  i2c_delay();
  I2C_SCL_HIGH();
  i2c_delay();
  acked = I2C_SDA_READ() ? 0u : 1u;   /* pulled low = ACK */
  I2C_SCL_LOW();
  i2c_delay();

  return acked;
}

/*  Receive one byte MSB-first, then NACK so the slave stops.
 *  SDA stays released for the whole data phase (the slave drives it); that
 *  only works with open-drain. */
static uint8_t i2c_read_byte(void)
{
  uint8_t v = 0u;

  I2C_SDA_HIGH();              /* release; slave drives from here */
  for (int bit = 7; bit >= 0; bit--)
  {
    i2c_delay();
    I2C_SCL_HIGH();
    i2c_delay();
    if (I2C_SDA_READ()) v |= (uint8_t)(1u << bit);   /* sample while SCL high */
    I2C_SCL_LOW();
  }
  i2c_delay();

  I2C_SDA_HIGH();              /* 9th clock: leave SDA high = NACK */
  i2c_delay();
  I2C_SCL_HIGH();
  i2c_delay();
  I2C_SCL_LOW();
  i2c_delay();

  return v;
}

/*  Read one register: write address -> write register pointer -> repeated
 *  START -> read. The repeated START is the direction change; a STOP cannot
 *  replace it. */
static uint8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *acked)
{
  uint8_t v;

  i2c_start();
  (void)i2c_byte((uint8_t)(addr << 1));            /* address, LSB 0 = write */
  (void)i2c_byte(reg);                             /* register pointer */
  i2c_start();                                     /* repeated START */
  *acked = i2c_byte((uint8_t)((addr << 1) | 1u));  /* same addr, LSB 1 = read */
  v = i2c_read_byte();
  i2c_stop();

  return v;
}

static void pca_write(uint8_t reg, uint8_t val)
{
  i2c_start();
  (void)i2c_byte((uint8_t)(PCA_ADDR << 1));
  (void)i2c_byte(reg);
  (void)i2c_byte(val);
  i2c_stop();
}

static uint8_t pca_read(uint8_t reg, uint8_t *acked)
{
  return i2c_read_reg(PCA_ADDR, reg, acked);
}


/* ---- 3. PCA9685 ------------------------------------------------------- */

/*  Bring the chip up. The four writes must stay in this order. */
static void pca_init(void)
{
  pca_write(PCA_MODE1, 0x10u);           /* SLEEP: PRESCALE only writable asleep */
  pca_write(PCA_PRESCALE, PCA_PRESCALE_50HZ);
  pca_write(PCA_MODE2, 0x04u);           /* totem-pole outputs (what a servo wants) */
  pca_write(PCA_MODE1, 0xA0u);           /* wake: auto-increment | RESTART */

  /*  Wait until RESTART clears itself — that means the oscillator is running.
   *
   *  Do not skip this. Writing 0x20 alone is what most snippets do, and it
   *  leaves you with a chip whose registers all read/write, whose PRESCALE
   *  holds, whose verify passes, and whose output pins are dead — with no
   *  error flag to look at. Sleep latches RESTART; something must write it
   *  back as 1 before the oscillator starts. */
  for (int i = 0; i < 1000; i++)
  {
    uint8_t acked = 0u;
    uint8_t m = pca_read(PCA_MODE1, &acked);

    if (acked != 0u && m == 0x20u) break;   /* RESTART cleared = awake */
  }
}

/*  Write one channel's pulse.
 *
 *  The chip does not "make a servo pulse". It counts 4096 steps per frame and
 *  turns a pin on at one step and off at another. A servo pulse is "on at
 *  step 0, off after N counts".
 *
 *  Six bytes go out in one transaction via auto-increment: after the pointer
 *  lands on ON_L, the next five bytes walk forward automatically. */
static uint8_t pca_write_pulse(uint8_t ch, uint16_t counts)
{
  uint8_t ok;

  i2c_start();
  ok  = i2c_byte((uint8_t)(PCA_ADDR << 1));
  ok &= i2c_byte((uint8_t)(PCA_LED0_ON_L + 4u * ch));
  ok &= i2c_byte(0x00u);                       /* ON_L = 0 */
  ok &= i2c_byte(0x00u);                       /* ON_H = 0 */
  ok &= i2c_byte((uint8_t)(counts & 0xFFu));   /* OFF_L    */
  ok &= i2c_byte((uint8_t)(counts >> 8));      /* OFF_H    */
  i2c_stop();

  return ok;
}

/*  Convert microseconds to counts and write the chip.
 *
 *  counts = us * 4096 / 20000; +10000 rounds to nearest. One count is
 *  4.883 us, so 1900 us becomes 389 counts and reads back as 1899 us. */
static void pca_set_us(axis_t a, uint16_t us)
{
  uint16_t counts = (uint16_t)(((uint32_t)us * 4096u + 10000u) / 20000u);

  /*  Dedupe is per-axis. A shared last-counts would compare axis 2 against
   *  axis 1's value and skip the second write whenever both axes hold the
   *  same angle (which is every cold start at centre). */
  if (counts == pca_last_counts[a]) return;

  /*  Only update the cache after an ACK. A dropped write must leave dedupe
   *  open so the next pass retries — otherwise the first lost packet is the
   *  last command that channel ever gets. */
  if (pca_write_pulse(pca_ch[a], counts) != 0u) pca_last_counts[a] = counts;
}

static void servo_us(axis_t a, uint16_t us)
{
  if (us < 500)  us = 500;      /* last-ditch rail; normal path never hits it */
  if (us > 2500) us = 2500;
  pca_set_us(a, us);
}


/* ---- 4. IMU ----------------------------------------------------------- */

static void imu_uart_init(uint32_t baud)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
  (void)RCC->APB2ENR;          /* read-back: wait for the clock before touching the peripheral */

  /*  PA10 = USART1_RX, CRH bits 11:8 (PA8 is nibble 0). Set as input with
   *  pull-up (CNF=10, MODE=00 -> 0x8).
   *
   *  Mask-and-rewrite; do not blast the whole register: PA13/PA14 are SWD in
   *  the same register, and clearing them bricks the chip for further flashes.
   *
   *  Pull-up rather than floating: an idle UART line should sit high, so a
   *  disconnected or unpowered module looks like "no signal" instead of a
   *  floating pin inventing random start bits. */
  GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x8u << 8);
  GPIOA->BSRR = (1u << 10);    /* ODR bit10 = 1 -> pull-up, not pull-down */

  USART1->BRR = 72000000u / baud;   /* 72 MHz / 115200, ~0.16% error */

  /*  Receiver only. Enabling TE would hand PA9 to the USART, and we have
   *  nothing to say on it. */
  USART1->CR1 = USART_CR1_UE | USART_CR1_RE;
}

static float imu_to_float(const uint8_t *p)
{
  union { uint32_t u; float f; } v;

  v.u = (uint32_t)p[0]
      | ((uint32_t)p[1] << 8)
      | ((uint32_t)p[2] << 16)
      | ((uint32_t)p[3] << 24);

  return v.f;
}

/*  Feed the parser one byte. Non-blocking, no software buffer — the USART's
 *  own data register is the buffer.
 *
 *  Frame layout:  7E 23 | LEN | FUNC | DATA... | SUM
 *    LEN  = whole-frame length (header through checksum)
 *    SUM  = (7E + 23 + LEN + FUNC + all DATA) & 0xFF
 *
 *  A two-byte header can still appear inside payload floats (~once per
 *  65536 bytes). What rejects a false header is LEN: a length taken from a
 *  random mid-float position almost never matches a legal frame size. */
static void imu_take_byte(uint8_t b)
{
  static uint8_t buf[IMU_MAX_LEN];
  static uint8_t state = 0u;
  static uint8_t idx   = 0u;
  static uint8_t flen  = 0u;
  static uint8_t func  = 0u;
  static uint8_t want  = 0u;

  switch (state)
  {
  case 0u:                                   /* hunting for HEAD1 */
    if (b == IMU_HDR1) state = 1u;
    break;

  case 1u:                                   /* hunting for HEAD2 */
    if      (b == IMU_HDR2) state = 2u;
    else if (b == IMU_HDR1) state = 1u;      /* 7E 7E 23 -> keep the later 7E */
    else                    state = 0u;
    break;

  case 2u:                                   /* length byte */
    if (b < 5u || b > IMU_MAX_LEN) { state = 0u; break; }
    flen  = b;
    state = 3u;
    break;

  case 3u:                                   /* function code */
    func  = b;
    idx   = 0u;
    want  = (uint8_t)(flen - 4u);            /* payload + checksum */
    state = 4u;
    break;

  default:                                   /* payload bytes */
    buf[idx++] = b;
    if (idx >= want)
    {
      uint8_t sum = (uint8_t)(IMU_HDR1 + IMU_HDR2 + flen + func);
      uint8_t i;

      for (i = 0u; i + 1u < want; i++) sum = (uint8_t)(sum + buf[i]);

      if (sum == buf[want - 1u] &&
          func == IMU_FUNC_EULER && flen == IMU_EULER_LEN)
      {
        /*  Three little-endian float32s in radians: roll, pitch, yaw.
         *  Only the first two are used. Yaw has no gravity reference and
         *  drifts, so it is useless for leveling. */
        imu_roll_deg  = imu_to_float(&buf[0]) * IMU_RAD_TO_DEG;
        imu_pitch_deg = imu_to_float(&buf[4]) * IMU_RAD_TO_DEG;
      }

      state = 0u;
    }
    break;
  }
}

/*  Pull one byte from USART1. Returns 1 if a byte was taken, 0 if idle.
 *
 *  SR is read once and kept: RXNE and the four error flags live in the same
 *  register, and on this chip the clear sequence is "read SR then read DR".
 *  The error flags must be sampled in that same SR read or they are gone.
 *
 *  Leave the DR as soon as it is read: USART1 has nowhere else to put a byte,
 *  so any arithmetic done here is a window in which the next byte is lost. */
static uint8_t imu_poll_rx(void)
{
  uint32_t sr = USART1->SR;

  if ((sr & USART_SR_RXNE) == 0u) return 0u;

  imu_take_byte((uint8_t)(USART1->DR & 0xFFu));
  return 1u;
}


/* ---- 5. Control law (PID) --------------------------------------------- */

/*  Angle (degrees) -> pulse width (us). Called once per axis with its own
 *  pid_axis_t. Full PID — not a staged lesson switch.
 *
 *      error = deg
 *      cmd   = sign * (Kp*e + Ki*i + Kd*de/dt)
 *      us    = CENTRE + cmd
 *
 *  dt from HAL_GetTick() between updates. First sample primes state (P only).
 *  Anti-windup freezes integration when saturated unless error drives back.
 *
 *  Two clamps, order fixed: ±max_us first, then SERVO_MIN/MAX.
 *  No setpoint argument — level is gravity (target 0°).
 *
 *  SIGN / Kp / Ki / Kd / MAX_US are hardware or tuning facts. Wrong SIGN is
 *  positive feedback. Set Ki or Kd to 0 to feel P or PD alone. */
static int32_t imu_axis_us(float deg, float sign, float kp, float ki, float kd,
                           uint16_t max_us, pid_axis_t *pid)
{
  const float error = deg;
  uint32_t now_ms = HAL_GetTick();
  float dt_s = 0.0f;
  float derivative = 0.0f;

  if (pid->primed != 0u)
  {
    uint32_t elapsed_ms = now_ms - pid->prev_ms;
    if (elapsed_ms > 0u)
    {
      dt_s = (float)elapsed_ms * 0.001f;
      derivative = (error - pid->prev_error) / dt_s;
    }
  }

  float p_term = kp * error;
  float d_term = kd * derivative;
  float i_term = ki * pid->integral;
  float unsaturated = sign * (p_term + i_term + d_term);

  if (pid->primed != 0u && dt_s > 0.0f)
  {
    const float max_f = (float)max_us;
    const uint8_t saturated_hi = (unsaturated >=  max_f) ? 1u : 0u;
    const uint8_t saturated_lo = (unsaturated <= -max_f) ? 1u : 0u;
    if ((saturated_hi == 0u && saturated_lo == 0u) ||
        (saturated_hi != 0u && (sign * error) < 0.0f) ||
        (saturated_lo != 0u && (sign * error) > 0.0f))
    {
      pid->integral += error * dt_s;
    }
    i_term = ki * pid->integral;
    unsaturated = sign * (p_term + i_term + d_term);
  }

  float cmd = unsaturated;
  if (cmd >  (float)max_us) cmd =  (float)max_us;
  if (cmd < -(float)max_us) cmd = -(float)max_us;

  pid->prev_error = error;
  pid->prev_ms    = now_ms;
  pid->primed     = 1u;

  int32_t us = (int32_t)IMU_SERVO_CENTRE + (int32_t)cmd;

  if (us < (int32_t)SERVO_MIN_US) us = (int32_t)SERVO_MIN_US;
  if (us > (int32_t)SERVO_MAX_US) us = (int32_t)SERVO_MAX_US;

  return us;
}


/* ---- 6. main ---------------------------------------------------------- */

int main(void)
{
  SystemClock_Config();        /* 72 MHz; CubeMX-generated, not expanded here */
  MX_GPIO_Init();

  /*  Bus first, then the chip, then the servos have a pulse source. */
  i2c_init();
  pca_init();

  /*  Hold both axes at centre until the IMU starts delivering angles. */
  servo_us(AXIS_ROLL,  IMU_SERVO_CENTRE);
  servo_us(AXIS_PITCH, IMU_SERVO_CENTRE);

  imu_uart_init(IMU_BAUD);

  for (;;)
  {
    /*  Only run the control law when UART has no byte waiting. USART1 has a
     *  one-register buffer; any time spent away from SR/DR is a window that
     *  can drop a byte.
     *
     *  That is also why this loop is control only: readback, bus scan, and
     *  LED display all live in other modes in the real main.c. */
    if (imu_poll_rx() == 0u)
    {
      servo_us(AXIS_ROLL,
               (uint16_t)imu_axis_us(imu_roll_deg,
                                     IMU_SERVO_ROLL_SIGN,
                                     IMU_SERVO_ROLL_KP, IMU_SERVO_ROLL_KI,
                                     IMU_SERVO_ROLL_KD, IMU_SERVO_ROLL_MAX_US,
                                     &pid_roll));
      servo_us(AXIS_PITCH,
               (uint16_t)imu_axis_us(imu_pitch_deg,
                                     IMU_SERVO_PITCH_SIGN,
                                     IMU_SERVO_PITCH_KP, IMU_SERVO_PITCH_KI,
                                     IMU_SERVO_PITCH_KD, IMU_SERVO_PITCH_MAX_US,
                                     &pid_pitch));
    }
  }
}

/* ===========================================================================
 *  Deliberately omitted here — present in Core/Src/main.c
 * ===========================================================================
 *
 *  Each item below exists to diagnose a problem that has already been solved
 *  once. A new minimal build can leave them all out, but knowing what is
 *  missing means knowing which eye you no longer have.
 *
 *    LED codes (imu_show)       When the IMU is silent, tells "no bytes at
 *                               all" apart from "bytes but no valid frame" —
 *                               they look identical and need opposite fixes.
 *    imu_frames / rx_bytes /    Counters for the above. Only frames frozen =
 *    imu_rx_errs                parser stuck; both frozen = wire or module.
 *    pca_readback()             Once a second, read the chip's pulse registers
 *                               and MODE1 to confirm writes landed and the
 *                               chip was not reset.
 *    pca_reinit()               Re-init if MODE1 falls back to 0x11.
 *    i2c_scan_bus()             Bus scan. Fastest answer when the address is
 *                               wrong.
 *    pca_selftest()             Startup chain of the above; failure flashes an
 *                               LED code.
 *    SERVO_SWEEP_TEST           Ignore the IMU; sweep servos alone. Separates
 *                               "servo path broken" from "control law broken".
 *    SERVO_HOLD_CENTRE          Both axes fixed at centre. Separates "servo
 *                               fighting the command" from "never commanded".
 *    pa10_watch()               First 200 ms after boot, watch the PA10 pin
 *                               itself. Splits "no bytes" into wiring vs module.
 *    HAL_Delay three flashes    A trusted landmark: clock is up and execution
 *                               reached this point.
 *
 *  Together those extras are ~700 lines — more than half of main.c. The
 *  control path itself is under 200 lines.
 */
