#ifndef USER_INC_INA226_H_
#define USER_INC_INA226_H_

#include <stdint.h>
#include "main.h"
#include "i2c.h"

/*
 * ina226.h
 * ------------------------------------------------------------
 * INA226 是电流/功率测量芯片。该模块负责把底层 I2C 访问封装成
 * 一个统一的接口，供电源管理和控制逻辑读取输入电压、电流和功率。
 *
 * 通过单独抽离此模块，确保所有电流采样逻辑集中在一个地方，
 * 不会分散在各个功能块中，便于调试和参数修正。
 */

#define INA226_ADDRESS 0x80U

#define INA226_CONFIG_REG          0x00U
#define INA226_SHUNT_VOLTAGE_REG   0x01U
#define INA226_BUS_VOLTAGE_REG     0x02U
#define INA226_POWER_REG           0x03U
#define INA226_CURRENT_REG         0x04U
#define INA226_CALIBRATION_REG     0x05U
#define INA226_MASK_ENABLE_REG     0x06U
#define INA226_ALERT_LIMIT_REG     0x07U
#define INA226_MANUFACTURER_ID_REG 0xFEU
#define INA226_DIE_ID_REG          0xFFU

#define INA226_CONFIG_DEFAULT      0x4527U
#define INA226_CALIBRATION_DEFAULT 0x0800U
#define INA226_SHUNT_RESISTANCE_MOHM 10U
#define INA226_CURRENT_LSB_UA      250U

/* 读取到的物理量以 V、A、W 为单位。 */
typedef struct
{
  float bus_voltage_v;
  float current_a;
  float power_w;
} INA226_Measurements_t;

void INA226_Init(void);
uint8_t INA226_IsConnected(void);
HAL_StatusTypeDef INA226_ReadMeasurements(INA226_Measurements_t *measurements);
uint16_t INA226_ReadRegister16(uint8_t reg);
uint8_t INA226_WriteRegister16(uint8_t reg, uint16_t value);
int16_t INA226_ReadCurrentRaw(void);
float INA226_ReadCurrentA(void);
float INA226_ReadBusVoltageV(void);
float INA226_ReadShuntVoltageMv(void);
float INA226_ReadPowerW(void);
uint16_t INA226_ReadManufacturerId(void);
uint16_t INA226_ReadDieId(void);

#endif /* USER_INC_INA226_H_ */
