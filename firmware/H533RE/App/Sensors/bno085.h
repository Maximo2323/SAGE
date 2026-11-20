#ifndef BNO085_H
#define BNO085_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "sh2_hal.h"
#include "constants.h"

/* The arrays have one entry for each report.
 * The index is HEADING_SOURCE_GAME_RV or HEADING_SOURCE_RV (see constants.h).
 * The two reports have different zero references. Do not compare their absolute values. */
typedef struct
{
  sh2_Hal_t          hal;  /* first member: sh2 gives &hal back to the HAL functions */
  I2C_HandleTypeDef *hi2c;
  uint8_t            addr;
  bool               initialized;
  bool               got_event;
  bool               reset_seen;    /* the sensor reported a reset: enable the reports again */
  uint32_t           reset_count;   /* number of sensor resets after the start               */
  float              yaw_deg[2];    /* newest heading, -180 to 180 degrees                   */
  float              yaw_start[2];  /* first heading after the start                         */
  uint32_t           t_ms[2];       /* HAL_GetTick() time of the newest report               */
  uint32_t           count[2];      /* number of reports received                            */
} Bno085;

bool  bno085_begin(Bno085 *b, I2C_HandleTypeDef *hi2c, uint8_t addr);
/* Read the reports that are ready. Call this function from the main loop, not from an interrupt. */
void  bno085_update(Bno085 *b);
/* True when a report from this source arrived in the last BNO_STALE_TIMEOUT_MS.
 * The age is signed: a report can be some milliseconds newer than now_ms, because the
 * main loop reads the time before it reads the sensor. */
static inline bool bno085_fresh(const Bno085 *b, int source, uint32_t now_ms)
{
  return b->initialized && b->count[source] != 0U
      && (int32_t)(now_ms - b->t_ms[source]) < (int32_t)BNO_STALE_TIMEOUT_MS;
}
/* Heading change since the first report of this source. Use it to compare the two sources. */
float bno085_relative_yaw(const Bno085 *b, int source);

#endif /* BNO085_H */
