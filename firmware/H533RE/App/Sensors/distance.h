/* Distance sensors: one interface for all sensor types.
 *
 * Example:
 *   uint16_t mm;
 *   if (distance_read(DIST_FRONT, &mm) == DIST_OK) { ... }
 *
 * At present all sensors are VL53L1X (TOF400C modules). To use an ultrasonic sensor for one
 * position, change distance.c only. The code that reads the distances does not change. */
#ifndef DISTANCE_H
#define DISTANCE_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* Sensor positions. TBD: confirm them when the sensors are installed. */
typedef enum
{
  DIST_FRONT = 0,   /* front of the car, points forward                         */
  DIST_RIGHT_LOW,   /* right side, near the ground                              */
  DIST_RIGHT_HIGH,  /* right side, points up, for landmarks above the low sensor */
  DIST_COUNT
} DistId;

typedef enum
{
  DIST_OK = 0,      /* a target at the given distance                            */
  DIST_NO_TARGET,   /* the sensor operates correctly and sees nothing in range   */
  DIST_NOT_VALID    /* fault, no sensor, or data older than DIST_STALE_MS        */
} DistStatus;

/* Start all sensors. Returns false if one or more sensors do not answer.
 * The sensors that answer continue to operate. */
bool       distance_init(I2C_HandleTypeDef *hi2c);
/* Read new measurements. Call this function from the main loop, not from an interrupt. */
void       distance_update(void);
/* Gives the newest distance in millimeters. Use *mm only when the result is DIST_OK. */
DistStatus distance_read(DistId id, uint16_t *mm);

#endif /* DISTANCE_H */
