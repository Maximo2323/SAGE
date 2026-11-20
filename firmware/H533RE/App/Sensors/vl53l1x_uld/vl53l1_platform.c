#include "vl53l1_platform.h"
#include "constants.h"

/* shortcut: all VL53L1X share one I2C bus, add the handle to dev if a second bus is used */
I2C_HandleTypeDef *vl53l1_hi2c;

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  return (HAL_I2C_Mem_Write(vl53l1_hi2c, dev, index, I2C_MEMADD_SIZE_16BIT, pdata,
                            (uint16_t)count, TOF_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  return (HAL_I2C_Mem_Read(vl53l1_hi2c, dev, index, I2C_MEMADD_SIZE_16BIT, pdata,
                           (uint16_t)count, TOF_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  uint8_t buf[2] = {(uint8_t)(data >> 8), (uint8_t)data};
  return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  uint8_t buf[4] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data};
  return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
  return VL53L1_ReadMulti(dev, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
  uint8_t buf[2];
  int8_t status = VL53L1_ReadMulti(dev, index, buf, 2);
  *pdata = (uint16_t)((buf[0] << 8) | buf[1]);
  return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
  uint8_t buf[4];
  int8_t status = VL53L1_ReadMulti(dev, index, buf, 4);
  *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
  return status;
}
