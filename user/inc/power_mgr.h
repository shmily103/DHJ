#ifndef USER_INC_POWER_MGR_H_
#define USER_INC_POWER_MGR_H_

#include <stdint.h>

/*
 * power_mgr.h
 * ------------------------------------------------------------
 * 电源管理模块。它负责把输入电源、CH224A 协商、INA226 测量与充电
 * 控制逻辑组织在一起，确保设备在不同外部供电条件下都能安全工作。
 *
 * 这个模块对上层暴露的是状态机和控制接口，而不是原始寄存器细节。
 * 这样 UI 和控制逻辑可以对“是否有有效输入、当前是 CC/CV 还是已完成”
 * 等高层状态进行判断，而无需关心协议和硬件细节。
 */

#define POWER_MGR_STATUS_CH224A_PRESENT 0x01U
#define POWER_MGR_STATUS_INA226_PRESENT 0x02U
#define POWER_MGR_STATUS_INPUT_VALID    0x08U
typedef enum
{
  POWER_MGR_CHARGE_DISABLED = 0,
  POWER_MGR_CHARGE_WAIT_SOURCE,
  POWER_MGR_CHARGE_NEGOTIATING,
  POWER_MGR_CHARGE_CC,
  POWER_MGR_CHARGE_CV,
  POWER_MGR_CHARGE_COMPLETE
} PowerMgr_ChargeState_t;

typedef struct
{
  uint8_t status_mask;
  float input_voltage_v;
  float input_current_a;
  float input_power_w;
  float source_max_current_a;
  uint16_t pps_voltage_mv;
  uint8_t ch224a_connected;
  uint8_t ina226_connected;
  uint8_t measurement_valid;
  uint8_t source_current_valid;
  uint32_t measurement_tick_ms;
  PowerMgr_ChargeState_t charge_state;
} PowerMgr_Status_t;

void PowerMgr_Init(void);
void PowerMgr_Task(void);
void PowerMgr_ChargeTask(void);
uint8_t PowerMgr_IsInputValid(void);
float PowerMgr_GetInputVoltageV(void);
float PowerMgr_GetInputCurrentA(void);
uint8_t PowerMgr_GetStatusMask(void);
void PowerMgr_GetStatus(PowerMgr_Status_t *status);
uint8_t PowerMgr_StartCalibrationCharge(float target_voltage_v, float current_a);
void PowerMgr_StopCalibrationCharge(void);
void PowerMgr_SetCalibrationMode(uint8_t active);
void PowerMgr_SetWeldInhibit(uint8_t active);

#endif /* USER_INC_POWER_MGR_H_ */
