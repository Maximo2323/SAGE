#ifndef TOF_H
#define TOF_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "distance.h"

/* One VL53L1X sensor. Use distance.h to read the sensors. */
typedef struct
{
  uint16_t      addr;
  GPIO_TypeDef *xshut_port;
  uint16_t      xshut_pin;
  bool          initialized;
  DistStatus    status;          /* status of the newest measurement */
  uint16_t      distance_mm;     /* valid when status is DIST_OK      */
  uint32_t      t_ms;            /* time of the newest measurement    */
  uint32_t      last_update_ms;
  uint16_t      update_interval_ms;
  uint16_t      max_range_mm;
} Tof;

void       tof_init(Tof *t, GPIO_TypeDef *xshut_port, uint16_t xshut_pin, uint16_t addr);
bool       tof_begin_all(I2C_HandleTypeDef *hi2c, Tof *tofs, uint8_t count);
void       tof_update(Tof *t);
DistStatus tof_read(const Tof *t, uint32_t now_ms, uint16_t *mm);

#endif /* TOF_H */
