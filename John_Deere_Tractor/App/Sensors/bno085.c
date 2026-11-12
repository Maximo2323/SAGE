#include "bno085.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "math_utils.h"
#include "constants.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f

static int i2c_open(sh2_Hal_t *self)
{
  Bno085 *b = (Bno085 *)self;
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};

  for (uint32_t i = 0; i < BNO_SOFTRESET_ATTEMPTS; i++)
  {
    if (HAL_I2C_Master_Transmit(b->hi2c, (uint16_t)(b->addr << 1), softreset_pkt,
                                sizeof(softreset_pkt), BNO_I2C_TIMEOUT_MS) == HAL_OK)
    {
      HAL_Delay(BNO_SOFTRESET_WAIT_MS);
      return 0;
    }
    HAL_Delay(BNO_SOFTRESET_RETRY_MS);
  }
  return -1;
}

static void i2c_close(sh2_Hal_t *self)
{
  (void)self;
}

/* shortcut: millisecond resolution, upgrade to a 1 MHz timer if sh2 timestamps are needed */
static uint32_t get_time_us(sh2_Hal_t *self)
{
  (void)self;
  return HAL_GetTick() * 1000U;
}

static int i2c_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us)
{
  Bno085 *b = (Bno085 *)self;
  uint8_t header[4];

  if (HAL_I2C_Master_Receive(b->hi2c, (uint16_t)(b->addr << 1), header, sizeof(header),
                             BNO_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  uint16_t packet_size = (uint16_t)(header[0] | (header[1] << 8)) & 0x7FFFU;
  if (packet_size == 0U || packet_size > len) { return 0; }

  /* The BNO sends the header again at the start of each read, so one read gets the full packet. */
  if (HAL_I2C_Master_Receive(b->hi2c, (uint16_t)(b->addr << 1), buf, packet_size,
                             BNO_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  *t_us = get_time_us(self);
  return packet_size;
}

static int i2c_write(sh2_Hal_t *self, uint8_t *buf, unsigned len)
{
  Bno085 *b = (Bno085 *)self;
  if (HAL_I2C_Master_Transmit(b->hi2c, (uint16_t)(b->addr << 1), buf, (uint16_t)len,
                              BNO_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }
  return (int)len;
}

static void async_handler(void *cookie, sh2_AsyncEvent_t *event)
{
  /* Do not send commands to the sensor from this callback. bno085_update() does that. */
  if (event->eventId == SH2_RESET) { ((Bno085 *)cookie)->reset_seen = true; }
}

static float quat_to_yaw_deg(float qi, float qj, float qk, float qr)
{
  const float ys = 2.0f * (qr * qk + qi * qj);
  const float yc = 1.0f - 2.0f * (qj * qj + qk * qk);
  return -atan2f(ys, yc) * RAD_TO_DEG;  /* ===== TBD: confirm on the car that a left turn increases the heading ===== */
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event)
{
  Bno085 *b = (Bno085 *)cookie;
  sh2_SensorValue_t value;

  if (sh2_decodeSensorEvent(&value, event) != SH2_OK) { return; }
  b->got_event = true;

  int   src;
  float yaw;
  if (value.sensorId == SH2_GAME_ROTATION_VECTOR)
  {
    const sh2_RotationVector_t *q = &value.un.gameRotationVector;
    src = HEADING_SOURCE_GAME_RV;
    yaw = quat_to_yaw_deg(q->i, q->j, q->k, q->real);
  }
  else if (value.sensorId == SH2_ROTATION_VECTOR)
  {
    const sh2_RotationVectorWAcc_t *q = &value.un.rotationVector;
    src = HEADING_SOURCE_RV;
    yaw = quat_to_yaw_deg(q->i, q->j, q->k, q->real);
  }
  else
  {
    return;
  }

  yaw = wrap180(yaw);
  if (b->count[src] == 0U) { b->yaw_start[src] = yaw; }
  b->yaw_deg[src] = yaw;
  b->t_ms[src]    = HAL_GetTick();
  b->count[src]++;
}

static bool start(Bno085 *b)
{
  if (HAL_I2C_IsDeviceReady(b->hi2c, (uint16_t)(b->addr << 1), 2, BNO_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }
  if (sh2_open(&b->hal, async_handler, b) != SH2_OK) { return false; }

  sh2_ProductIds_t ids;
  if (sh2_getProdIds(&ids) != SH2_OK) { return false; }

  sh2_setSensorCallback(sensor_handler, b);
  return true;
}

static bool enable_reports(void)
{
  sh2_SensorConfig_t config = {0};

  config.reportInterval_us = BNO_GAME_RV_INTERVAL_US;
  if (sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config) != SH2_OK) { return false; }

  config.reportInterval_us = BNO_RV_INTERVAL_US;
  return sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) == SH2_OK;
}

bool bno085_begin(Bno085 *b, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
  b->hal.open      = i2c_open;
  b->hal.close     = i2c_close;
  b->hal.read      = i2c_read;
  b->hal.write     = i2c_write;
  b->hal.getTimeUs = get_time_us;
  b->hi2c          = hi2c;
  b->initialized   = false;
  b->reset_seen    = false;
  b->reset_count   = 0;
  for (int i = 0; i < 2; i++)
  {
    b->yaw_deg[i]   = 0.0f;
    b->yaw_start[i] = 0.0f;
    b->t_ms[i]      = 0;
    b->count[i]     = 0;
  }

  const uint8_t candidates[2] = {addr, (uint8_t)(addr ^ 0x01U)};

  for (uint32_t attempt = 0; attempt < BNO_BEGIN_ATTEMPTS; attempt++)
  {
    HAL_Delay(BNO_BOOT_DELAY_MS);

    bool started = false;
    for (uint32_t i = 0; i < 2U; i++)
    {
      b->addr = candidates[i];
      if (start(b))
      {
        started = true;
        break;
      }
    }
    if (!started) { continue; }

    HAL_Delay(BNO_REPORT_DELAY_MS);
    if (!enable_reports()) { continue; }

    b->reset_seen  = false;  /* the reset at the start is normal */
    b->initialized = true;
    return true;
  }
  return false;
}

void bno085_update(Bno085 *b)
{
  if (!b->initialized) { return; }

  /* After a reset the sensor sends no reports. Enable them again. */
  if (b->reset_seen)
  {
    b->reset_seen = false;
    b->reset_count++;
    enable_reports();
  }

  do
  {
    b->got_event = false;
    sh2_service();
  } while (b->got_event);
}

bool bno085_fresh(const Bno085 *b, int source, uint32_t now_ms)
{
  return b->initialized && b->count[source] != 0U
      && (now_ms - b->t_ms[source]) < BNO_STALE_TIMEOUT_MS;
}

float bno085_relative_yaw(const Bno085 *b, int source)
{
  return wrap180(b->yaw_deg[source] - b->yaw_start[source]);
}
