/* PC test only: this file replaces Core/Inc/main.h, so that heading_hold.c compiles without the HAL. */
#ifndef MAIN_H_STUB
#define MAIN_H_STUB

#include <stdint.h>

typedef struct I2C_HandleTypeDef I2C_HandleTypeDef;
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

#endif /* MAIN_H_STUB */
