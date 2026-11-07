#ifndef USER_INC_CONTROL_H_
#define USER_INC_CONTROL_H_

#include <stdint.h>
#include "main.h"
#include "tft_ui.h"

/*
 * control.h
 * ------------------------------------------------------------
 * 控制核心头文件，负责系统的关键闭环逻辑：
 *  - 采样电池/电容组电压；
 *  - 监测电流和过压状态；
 *  - 控制均衡开关与点焊功率输出；
 *  - 处理触摸事件及校准过程。
 *
 * 设计目标是让上层 UI 与底层硬件之间保持清晰边界：
 *  - TftUi 模块只负责显示和用户交互；
 *  - Control 模块负责安全决策和动作执行；
 *  - 主循环通过 Control_Task() 周期运行状态机，以保证实时性。
 */

/* 电压采样、均衡、点焊和校准控制；Task 需由主循环周期调用。 */
void Control_Init(void);
void Control_Task(void);
void Control_NotifySettingsChanged(void);
void Control_HandleTouch(void);
uint8_t Control_StartCalibration(TftUiCalibrationType_t type);
void Control_CancelCalibration(void);
void Control_PowerFailShutdown(void);
float Control_GetCellVoltage(uint8_t index);
float Control_GetAdcCellVoltage(uint8_t index);
uint8_t Control_GetTotalVoltage(float *total_voltage_v);
uint8_t Control_GetOvervoltageCell(uint8_t *cell_index,
                                   float *cell_voltage_v,
                                   float *limit_voltage_v);
float Control_GetCurrentA(void);
uint8_t Control_IsChargeSafe(void);
uint8_t Control_GetBalanceMask(void);

#endif /* USER_INC_CONTROL_H_ */
