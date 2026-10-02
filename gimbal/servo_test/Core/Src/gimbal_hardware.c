/**
 * @file gimbal_hardware.c
 * @brief Gimbal drivers behind the public getter/setter API.
 *
 * I2C bit-bang (PB6/PB7) -> PCA9685 servos; USART1 RX (PA10) -> Yahboom IMU.
 * Control code must not include this file's internals — use gimbal_hardware.h.
 */

#include "gimbal_hardware.h"
#include "main.h"

/* ---- Servo pulse width -------------------------------------------------- */
#define SERVO_MIN_US          1000
#define SERVO_MAX_US          2000
#define SERVO_BACKSTOP_MIN     500
#define SERVO_BACKSTOP_MAX    2500
#define IMU_SERVO_CENTRE  ((SERVO_MIN_US + SERVO_MAX_US) / 2)

/* ---- PCA9685 ------------------------------------------------------------ */
#define PCA_ADDR              0x40u
#define PCA_MODE1             0x00u
#define PCA_MODE2             0x01u
#define PCA_LED0_ON_L         0x06u
#define PCA_PRESCALE          0xFEu
#define PCA_PRESCALE_50HZ       121u
#define PCA_MODE1_WAKE        0xA0u
#define PCA_MODE2_OUTDRV      0x04u
#define PCA_MODE1_SLEEP       0x10u

/* ---- IMU (USART, not I2C) ----------------------------------------------- */
#define IMU_BAUD            115200u
#define IMU_HDR1              0x7Eu
#define IMU_HDR2              0x23u
#define IMU_FUNC_EULER        0x26u
#define IMU_EULER_LEN           17u
#define IMU_LEN_MIN              5u
#define IMU_LEN_MAX             64u
#define IMU_DATA_MAX  (IMU_LEN_MAX - 5u)
#define IMU_RAD_TO_DEG    57.29578f

#define I2C_SCL_HIGH()  (GPIOB->BSRR = (1u << 6))
#define I2C_SCL_LOW()   (GPIOB->BRR  = (1u << 6))
#define I2C_SDA_HIGH()  (GPIOB->BSRR = (1u << 7))
#define I2C_SDA_LOW()   (GPIOB->BRR  = (1u << 7))
#define I2C_SDA_READ()  ((GPIOB->IDR & (1u << 7)) != 0u)

typedef enum
{
  HW_AXIS_ROLL = 0,
  HW_AXIS_PITCH,
  HW_AXIS_COUNT
} hw_axis_t;

typedef enum
{
  ST_HDR1 = 0,
  ST_HDR2,
  ST_LEN,
  ST_FUNC,
  ST_DATA,
  ST_SUM
} imu_state_t;

static const uint8_t pca_ch[HW_AXIS_COUNT] = { 0u, 8u };
static uint16_t pca_last_counts[HW_AXIS_COUNT] = { 0xFFFFu, 0xFFFFu };

static float imu_roll_deg;
static float imu_pitch_deg;
static volatile uint8_t imu_frame_ready;

static float    dt_seconds;
static uint32_t last_frame_ms;
static uint8_t  dt_primed;

static imu_state_t imu_st;
static uint8_t     imu_len;
static uint8_t     imu_func;
static uint8_t     imu_idx;
static uint8_t     imu_sum;
static uint8_t     imu_buf[IMU_DATA_MAX];

/* ---- Unused first-order LPF (math present; not wired into getters) ------ */
#define IMU_LPF_ALPHA  0.2f

static float lpf_roll;
static float lpf_pitch;
static uint8_t lpf_primed;

static void imu_lpf_update(float roll_raw, float pitch_raw) __attribute__((unused));
static void imu_lpf_update(float roll_raw, float pitch_raw)
{
  if (lpf_primed == 0u)
  {
    lpf_roll = roll_raw;
    lpf_pitch = pitch_raw;
    lpf_primed = 1u;
    return;
  }
  lpf_roll  += IMU_LPF_ALPHA * (roll_raw  - lpf_roll);
  lpf_pitch += IMU_LPF_ALPHA * (pitch_raw - lpf_pitch);
}

/* ---- Bit-bang I2C ------------------------------------------------------- */

static void i2c_delay(void)
{
  for (volatile int i = 0; i < 60; i++) { }
}

static void i2c_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  (void)RCC->APB2ENR;

  GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0x77u << 24);

  I2C_SCL_HIGH();
  I2C_SDA_HIGH();
}

static void i2c_start(void)
{
  I2C_SDA_HIGH();
  I2C_SCL_HIGH();
  i2c_delay();
  I2C_SDA_LOW();
  i2c_delay();
  I2C_SCL_LOW();
}

static void i2c_stop(void)
{
  I2C_SDA_LOW();
  I2C_SCL_HIGH();
  i2c_delay();
  I2C_SDA_HIGH();
  i2c_delay();
}

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

  I2C_SDA_HIGH();
  i2c_delay();
  I2C_SCL_HIGH();
  i2c_delay();
  acked = I2C_SDA_READ() ? 0u : 1u;
  I2C_SCL_LOW();
  i2c_delay();

  return acked;
}

/* ---- PCA9685 ------------------------------------------------------------ */

static void pca_write(uint8_t reg, uint8_t val)
{
  i2c_start();
  (void)i2c_byte((uint8_t)(PCA_ADDR << 1));
  (void)i2c_byte(reg);
  (void)i2c_byte(val);
  i2c_stop();
}

static void pca_init(void)
{
  pca_write(PCA_MODE1, PCA_MODE1_SLEEP);
  pca_write(PCA_PRESCALE, PCA_PRESCALE_50HZ);
  pca_write(PCA_MODE2, PCA_MODE2_OUTDRV);
  pca_write(PCA_MODE1, PCA_MODE1_WAKE);
  HAL_Delay(1);
}

static uint8_t pca_write_pulse(uint8_t ch, uint16_t counts)
{
  uint8_t ok;

  i2c_start();
  ok  = i2c_byte((uint8_t)(PCA_ADDR << 1));
  ok &= i2c_byte((uint8_t)(PCA_LED0_ON_L + 4u * ch));
  ok &= i2c_byte(0x00u);
  ok &= i2c_byte(0x00u);
  ok &= i2c_byte((uint8_t)(counts & 0xFFu));
  ok &= i2c_byte((uint8_t)(counts >> 8));
  i2c_stop();

  return ok;
}

static void pca_set_us(hw_axis_t a, uint16_t us)
{
  uint16_t counts = (uint16_t)(((uint32_t)us * 4096u + 10000u) / 20000u);

  if (counts == pca_last_counts[a]) return;

  if (pca_write_pulse(pca_ch[a], counts) != 0u) pca_last_counts[a] = counts;
}

static void servo_us(hw_axis_t a, uint16_t us)
{
  if (us < SERVO_BACKSTOP_MIN) us = SERVO_BACKSTOP_MIN;
  if (us > SERVO_BACKSTOP_MAX) us = SERVO_BACKSTOP_MAX;
  pca_set_us(a, us);
}

/* ---- IMU UART ----------------------------------------------------------- */

static void imu_uart_init(uint32_t baud)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
  (void)RCC->APB2ENR;

  GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x8u << 8);
  GPIOA->BSRR = (1u << 10);

  USART1->BRR = 72000000u / baud;
  USART1->CR1 = USART_CR1_UE | USART_CR1_RE;
}

static float imu_f32_le(const uint8_t *p)
{
  union { uint32_t u; float f; } v;

  v.u = (uint32_t)p[0]
      | ((uint32_t)p[1] << 8)
      | ((uint32_t)p[2] << 16)
      | ((uint32_t)p[3] << 24);

  if (v.f != v.f) return 0.0f;
  return v.f;
}

static void imu_take_byte(uint8_t b)
{
  switch (imu_st)
  {
    case ST_HDR1:
      if (b == IMU_HDR1) imu_st = ST_HDR2;
      break;

    case ST_HDR2:
      if (b == IMU_HDR2)      imu_st = ST_LEN;
      else if (b == IMU_HDR1) imu_st = ST_HDR2;
      else                    imu_st = ST_HDR1;
      break;

    case ST_LEN:
      imu_len = b;
      if (imu_len < IMU_LEN_MIN || imu_len > IMU_LEN_MAX) { imu_st = ST_HDR1; break; }
      imu_sum = (uint8_t)(IMU_HDR1 + IMU_HDR2 + imu_len);
      imu_st = ST_FUNC;
      break;

    case ST_FUNC:
      imu_func = b;
      imu_sum  = (uint8_t)(imu_sum + b);
      imu_idx  = 0u;
      imu_st   = (imu_len > IMU_LEN_MIN) ? ST_DATA : ST_SUM;
      break;

    case ST_DATA:
      imu_buf[imu_idx++] = b;
      imu_sum = (uint8_t)(imu_sum + b);
      if (imu_idx >= (uint8_t)(imu_len - 5u)) imu_st = ST_SUM;
      break;

    default:  /* ST_SUM */
      imu_st = ST_HDR1;
      if (b != imu_sum) return;
      if (imu_func != IMU_FUNC_EULER) return;
      if (imu_len != IMU_EULER_LEN) return;

      imu_roll_deg  = imu_f32_le(&imu_buf[0]) * IMU_RAD_TO_DEG;
      imu_pitch_deg = imu_f32_le(&imu_buf[4]) * IMU_RAD_TO_DEG;
#if 0
      /* Wire later if filtered angles are desired. Getters stay raw. */
      imu_lpf_update(imu_roll_deg, imu_pitch_deg);
#endif
      imu_frame_ready = 1u;
      break;
  }
}

static uint8_t imu_poll_rx(void)
{
  uint32_t sr = USART1->SR;

  if ((sr & USART_SR_RXNE) == 0u) return 0u;

  imu_take_byte((uint8_t)(USART1->DR & 0xFFu));
  return 1u;
}

/* ---- Public API --------------------------------------------------------- */

void gimbal_hardware_init(void)
{
  i2c_init();
  pca_init();

  servo_us(HW_AXIS_ROLL,  (uint16_t)IMU_SERVO_CENTRE);
  servo_us(HW_AXIS_PITCH, (uint16_t)IMU_SERVO_CENTRE);
  HAL_Delay(500);

  imu_uart_init(IMU_BAUD);

  dt_seconds = 0.0f;
  dt_primed  = 0u;
  last_frame_ms = HAL_GetTick();
}

int gimbal_hardware_update(void)
{
  for (int n = 0; n < (int)IMU_LEN_MAX && imu_poll_rx() != 0u; n++) { }

  if (imu_frame_ready == 0u) return 0;

  imu_frame_ready = 0u;

  {
    uint32_t now = HAL_GetTick();
    if (dt_primed != 0u)
    {
      uint32_t elapsed = now - last_frame_ms;
      dt_seconds = (elapsed > 0u) ? ((float)elapsed * 0.001f) : 0.0f;
    }
    else
    {
      dt_seconds = 0.0f;
      dt_primed = 1u;
    }
    last_frame_ms = now;
  }

  return 1;
}

float get_roll_angle(void)
{
  return imu_roll_deg;
}

float get_pitch_angle(void)
{
  return imu_pitch_deg;
}

float get_dt_seconds(void)
{
  return dt_seconds;
}

void set_servo_pulse(int axis, int pulse_us)
{
  int us = pulse_us;

  if (us < (int)SERVO_BACKSTOP_MIN) us = (int)SERVO_BACKSTOP_MIN;
  if (us > (int)SERVO_BACKSTOP_MAX) us = (int)SERVO_BACKSTOP_MAX;

  if (axis == AXIS_ROLL)
    servo_us(HW_AXIS_ROLL, (uint16_t)us);
  else if (axis == AXIS_PITCH)
    servo_us(HW_AXIS_PITCH, (uint16_t)us);
}
