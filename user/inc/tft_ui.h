#ifndef USER_INC_TFT_UI_H_
#define USER_INC_TFT_UI_H_

#include <stdint.h>
#include "main.h"
#include "encoder.h"

/*
 * tft_ui.h
 * ------------------------------------------------------------
 * 用户界面与数据模型定义。
 *
 * 该模块将显示层、菜单参数、状态反馈与 Flash 持久化数据统一成
 * 一个清晰接口。上层逻辑通过 TftUi_Settings、TftUi_Task() 和
 * 各类 Set/Get API 更新界面，而无需直接接触底层屏幕驱动细节。
 */

#define TFT_UI_MENU_ITEMS 13

/* 界面当前页面及工作状态，用于在控制逻辑与显示层之间同步状态。 */
typedef enum
{
  TFT_UI_SCREEN_HOME = 0,
  TFT_UI_SCREEN_MENU = 1,
  TFT_UI_SCREEN_CALIBRATION = 2
} TftUiScreen_t;

typedef enum
{
  TFT_UI_STATE_READY = 0,
  TFT_UI_STATE_CHARGING = 1,
  TFT_UI_STATE_BALANCING = 2,
  TFT_UI_STATE_WELDING = 3
} TftUiWorkState_t;

typedef enum
{
  TFT_UI_SAFETY_ALERT_CONTROL = 0,
  TFT_UI_SAFETY_ALERT_COUNT
} TftUiSafetyAlertSource_t;

/* 脉冲/间隔单位为 ms；触发等待以 100 ms 为一个计数单位。 */
typedef struct
{
  uint16_t pulse1_ms;
  uint16_t pulse2_ms;
  uint16_t interval_ms;
  uint16_t trigger_time_100ms;
  uint8_t capacitor_series;
  float cell_voltage;
  float balance_delta_v;
  float charge_current_a;
  uint8_t charge_enable;
  uint8_t brightness_percent;
} TftUiSettings_t;

typedef enum
{
  TFT_UI_CALIBRATION_CAPACITY = 0,
  TFT_UI_CALIBRATION_VOLTAGE_DROP
} TftUiCalibrationType_t;

typedef struct
{
  float capacitance_f;
  float voltage_drop_v;
} TftUiCalibrationData_t;

void TftUi_Init(void);
/* 在主循环中调用：处理消息超时、定时刷新和界面重绘。 */
void TftUi_Task(void);
void TftUi_ShowHome(void);
void TftUi_ShowMenu(void);
void TftUi_SetScreen(TftUiScreen_t screen);
void TftUi_SetWorkState(TftUiWorkState_t state);
TftUiWorkState_t TftUi_GetWorkState(void);
void TftUi_GetSettings(TftUiSettings_t *settings);
void TftUi_SetCounters(uint32_t total_count, uint32_t session_count);
void TftUi_GetCounters(uint32_t *total_count, uint32_t *session_count);
uint16_t TftUi_GetTriggerTime100ms(void);
void TftUi_NotifyMessage(const char *msg, uint16_t duration_ms);
void TftUi_SetSafetyAlert(TftUiSafetyAlertSource_t source, uint8_t active, const char *msg);
void TftUi_ProcessEncoderEvent(EncoderEvent_t event);
void TftUi_ProcessClick(void);
void TftUi_DrawHome(void);
void TftUi_DrawMenu(void);
void TftUi_RefreshDisplay(void);
uint32_t TftUi_LoadPersistentCount(void);
void TftUi_SavePersistentCount(uint32_t total_count);
uint8_t TftUi_SavePendingCount(void);
void TftUi_SetWeldEnergy(float energy_j);
void TftUi_SetCalibrationStatus(const char *status);
void TftUi_SetCalibrationActive(uint8_t active);
void TftUi_LoadCalibrationData(TftUiCalibrationData_t *data);
uint8_t TftUi_SaveCalibrationData(const TftUiCalibrationData_t *data);

#endif /* USER_INC_TFT_UI_H_ */
