#ifndef USER_INC_CH224A_H_
#define USER_INC_CH224A_H_

#include <stdint.h>
#include <string.h>
#include "main.h"
#include "i2c.h"

/*
 * ch224a.h
 * ------------------------------------------------------------
 * CH224A 是一颗 USB-PD / PPS 协议芯片，负责供电端的设备检测和
 * 电压/功率协商。
 *
 * 该驱动对上层隐藏了 I2C 地址选择和寄存器位定义，外部只需要
 * 调用检测、读取最大可用 PDO 电流和请求 PPS 电压即可。将复杂
 * 的协议细节封装在实现文件中，可减少系统中其他模块对协议格式
 * 的依赖，并增加可维护性。
 */

#define CH224A_ADDR_1             0x22U
#define CH224A_ADDR_2             0x23U
#define CH224A_REG_VOLTAGE_CTRL   0x0AU
#define CH224A_REG_STATUS         0x09U
#define CH224A_REG_PDO_CURRENT    0x50U
#define CH224A_REG_PPS_VOLTAGE    0x53U
#define CH224A_PPS_MIN_VOLTAGE_MV 3300U
#define CH224A_PPS_MAX_VOLTAGE_MV 12000U
#define CH224A_PDO_CURRENT_LSB_MA 50U
#define CH224A_REG_POWER_DATA_START 0x60U
#define CH224A_POWER_DATA_SIZE      48U

typedef enum
{
  DEVICE_STATE_DISCONNECTED = 0,
  DEVICE_STATE_CONNECTED,
  DEVICE_STATE_NEGOTIATING
} device_state_t;

typedef struct
{
  uint8_t i2c_addr;
  uint8_t protocol_status;
  device_state_t state;
} ch224a_device_t;

void CH224A_Init(void);
uint8_t CH224A_IsConnected(void);
uint8_t CH224A_DetectDevice(void);
HAL_StatusTypeDef CH224A_ReadPdoMaxCurrentMa(uint16_t *current_ma);
HAL_StatusTypeDef CH224A_ReadPowerData(uint8_t *data);
HAL_StatusTypeDef CH224A_RequestPPSVoltage(uint8_t addr, uint16_t voltage_mv);

extern ch224a_device_t ch224a_dev;

#endif /* USER_INC_CH224A_H_ */
