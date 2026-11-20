#include "tof.h"
#include "VL53L1X_api.h"
#include "constants.h"
#include "pins.h"

void tof_init(Tof *t, GPIO_TypeDef *xshut_port, uint16_t xshut_pin, uint16_t addr)
{
  t->addr               = addr;
  t->xshut_port         = xshut_port;
  t->xshut_pin          = xshut_pin;
  t->initialized        = false;
  t->status             = DIST_NOT_VALID;
  t->distance_mm        = 0;
  t->t_ms               = 0;
  t->last_update_ms     = 0;
  t->update_interval_ms = TOF_INTER_MEASUREMENT_MS;
  t->max_range_mm       = TOF_MAX_RANGE_MM;
}

static bool wait_boot(void)
{
  uint32_t start  = HAL_GetTick();
  uint8_t  booted = 0;

  while (HAL_GetTick() - start < TOF_BOOT_TIMEOUT_MS)
  {
    if (VL53L1X_BootState(TOF_DEFAULT_ADDR, &booted) == 0 && booted) { return true; }
    HAL_Delay(TOF_BOOT_POLL_MS);
  }
  return false;
}

static bool begin_one(Tof *t)
{
  HAL_GPIO_WritePin(t->xshut_port, t->xshut_pin, GPIO_PIN_SET);

  bool ok = wait_boot();
  if (ok && t->addr != TOF_DEFAULT_ADDR)
  {
    ok = VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, (uint8_t)t->addr) == 0;
  }
  ok = ok && VL53L1X_SensorInit(t->addr) == 0
          && VL53L1X_SetDistanceMode(t->addr, TOF_DISTANCE_MODE) == 0
          && VL53L1X_SetTimingBudgetInMs(t->addr, TOF_TIMING_BUDGET_MS) == 0
          && VL53L1X_SetInterMeasurementInMs(t->addr, TOF_INTER_MEASUREMENT_MS) == 0
          && VL53L1X_StartRanging(t->addr) == 0;

  if (!ok)
  {
    /* Keep a failed sensor off, so it does not block the default address for the next one. */
    HAL_GPIO_WritePin(t->xshut_port, t->xshut_pin, GPIO_PIN_RESET);
    return false;
  }

  t->initialized        = true;
  t->update_interval_ms = TOF_INTER_MEASUREMENT_MS;
  t->last_update_ms     = HAL_GetTick();
  return true;
}

bool tof_begin_all(I2C_HandleTypeDef *hi2c, Tof *tofs, uint8_t count)
{
  vl53l1_hi2c = hi2c;

  for (uint8_t i = 0; i < count; i++)
  {
    tofs[i].initialized = false;
    HAL_GPIO_WritePin(tofs[i].xshut_port, tofs[i].xshut_pin, GPIO_PIN_RESET);
  }
  HAL_Delay(TOF_XSHUT_RESET_MS);

  bool all_ok = true;
  for (uint8_t i = 0; i < count; i++)
  {
    if (!begin_one(&tofs[i])) { all_ok = false; }
  }
  return all_ok;
}

void tof_update(Tof *t)
{
  if (!t->initialized) { return; }

  uint32_t now = HAL_GetTick();
  if (now - t->last_update_ms < t->update_interval_ms) { return; }
  t->last_update_ms = now;

  uint8_t ready = 0;
  if (VL53L1X_CheckForDataReady(t->addr, &ready) != 0)
  {
    t->status = DIST_NOT_VALID;  /* I2C failure */
    return;
  }
  if (!ready) { return; }        /* no new measurement: keep the last one and its time */

  uint8_t  range_status = 0;
  uint16_t raw          = 0;
  const bool i2c_ok = VL53L1X_GetRangeStatus(t->addr, &range_status) == 0
                   && VL53L1X_GetDistance(t->addr, &raw) == 0;
  VL53L1X_ClearInterrupt(t->addr);

  if (!i2c_ok)
  {
    t->status = DIST_NOT_VALID;
    return;
  }
  t->t_ms = now;

  /* ===== TBD: confirm these range status values with the real sensor. =====
   * 0 = valid. 2 = signal too weak. 4 = out of bounds. 7 = wrap-around (target too far). */
  switch (range_status)
  {
    case 0:
      if (raw == 0U)                  { t->status = DIST_NOT_VALID; }
      else if (raw > t->max_range_mm) { t->status = DIST_NO_TARGET; }
      else                            { t->status = DIST_OK; t->distance_mm = raw; }
      break;
    case 2:
    case 4:
    case 7:
      t->status = DIST_NO_TARGET;
      break;
    default:
      t->status = DIST_NOT_VALID;
      break;
  }
}

DistStatus tof_read(const Tof *t, uint32_t now_ms, uint16_t *mm)
{
  if (!t->initialized || (now_ms - t->t_ms) >= DIST_STALE_MS) { return DIST_NOT_VALID; }
  *mm = t->distance_mm;
  return t->status;
}
