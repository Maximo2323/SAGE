#include "distance.h"
#include "tof.h"
#include "pins.h"

static Tof tofs[DIST_COUNT];

bool distance_init(I2C_HandleTypeDef *hi2c)
{
  /* ===== TBD: the XSHUT pins are placeholders. CubeMX does not know them.
   * When the pins are final, assign them in CubeMX and remove this GPIO setup. ===== */
  static GPIO_TypeDef *const port[DIST_COUNT] = {TOF_XSHUT_FRONT_PORT, TOF_XSHUT_RIGHT_LOW_PORT, TOF_XSHUT_RIGHT_HIGH_PORT};
  static const uint16_t      pin[DIST_COUNT]  = {TOF_XSHUT_FRONT_PIN,  TOF_XSHUT_RIGHT_LOW_PIN,  TOF_XSHUT_RIGHT_HIGH_PIN};
  static const uint16_t      addr[DIST_COUNT] = {TOF_ADDR_1, TOF_ADDR_2, TOF_ADDR_3};

  TOF_XSHUT_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  for (int i = 0; i < DIST_COUNT; i++)
  {
    HAL_GPIO_WritePin(port[i], pin[i], GPIO_PIN_RESET);
    gpio.Pin = pin[i];
    HAL_GPIO_Init(port[i], &gpio);
    tof_init(&tofs[i], port[i], pin[i], addr[i]);
  }
  return tof_begin_all(hi2c, tofs, DIST_COUNT);
}

void distance_update(void)
{
  for (int i = 0; i < DIST_COUNT; i++) { tof_update(&tofs[i]); }
}

DistStatus distance_read(DistId id, uint16_t *mm)
{
  return tof_read(&tofs[id], HAL_GetTick(), mm);
}
