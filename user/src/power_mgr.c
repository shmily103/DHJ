#include "power_mgr.h"
#include <stddef.h>
#include "ch224a.h"
#include "control.h"
#include "ina226.h"
#include "tim.h"
#include "tft_ui.h"

/*
 * power_mgr.c
 * ------------------------------------------------------------
 * 电源管理状态机实现。
 *
 * 这个模块把外部输入源、CH224A 协商、INA226 采样和输出使能一起
 * 组织成一个统一的 charge state machine。系统中所有的充电、
 * 停止、过压判断和校准相关逻辑，最终都要流经这里，使得安全
 * 策略集中、易于维护并减少各模块直接干扰。
 */

#define POWER_MGR_SAMPLE_PERIOD_MS          20U
#define POWER_MGR_DETECT_PERIOD_MS         100U
#define POWER_MGR_DISCONNECTED_RETRY_MS   1000U
#define POWER_MGR_PPS_DETECTION_TIMEOUT_MS 1000U
#define POWER_MGR_DEVICE_RETRY_PERIOD_MS  1000U
#define POWER_MGR_NEGATIVE_CURRENT_MS      100U
#define POWER_MGR_MEASUREMENT_MAX_AGE_MS   100U
#define POWER_MGR_PPS_SETTLE_MS             100U
#define POWER_MGR_PPS_RAMP_PERIOD_MS       500U
#define POWER_MGR_CURRENT_DEADBAND_A       0.10f
#define POWER_MGR_MAX_PPS_APDOS (CH224A_POWER_DATA_SIZE / 4U)
#define POWER_MGR_SOURCE_CURRENT_MARGIN     0.80f
#define POWER_MGR_SOURCE_OVERCURRENT_MARGIN 0.90f
#define POWER_MGR_COMPLETE_RECHARGE_HYST_V  0.10f
#define POWER_MGR_OVERVOLTAGE_RECOVERY_HYST_V 0.01f
#define POWER_MGR_OVERVOLTAGE_RECOVERY_MS    500U
#define POWER_MGR_PPS_STEP_MV              100U
#define POWER_MGR_HARDWARE_MIN_MV CH224A_PPS_MIN_VOLTAGE_MV
#define POWER_MGR_PPS_MAX_MV              11500U

static PowerMgr_Status_t g_status = {0};
static uint32_t g_last_sample_ms = 0U;
static uint32_t g_last_detect_ms = 0U;
static uint32_t g_last_ina_retry_ms = 0U;
static uint32_t g_negative_since_ms = 0U;
static uint32_t g_last_current_sample_ms = 0U;
static uint32_t g_last_ramp_sample_ms = 0U;
static uint32_t g_pps_request_ms = 0U;
static uint32_t g_pps_settle_since_ms = 0U;
static uint32_t g_pps_settle_sample_ms = 0U;
static uint32_t g_last_ramp_ms = 0U;
static uint32_t g_source_connected_since_ms = 0U;
static uint32_t g_cell_overvoltage_recovery_since_ms = 0U;
static uint16_t g_initial_voltage_mv = 0U;
static uint8_t g_charge_output_enabled = 0U;
static uint8_t g_weld_inhibit = 0U;
static uint8_t g_calibration_inhibit = 0U;
static uint8_t g_calibration_charge_active = 0U;
static uint8_t g_negotiation_started = 0U;
static uint8_t g_negative_current_tracking = 0U;
static uint8_t g_pps_settle_tracking = 0U;
static uint8_t g_cell_overvoltage_latched = 0U;
static uint8_t g_cell_overvoltage_recovery_tracking = 0U;
static uint8_t g_pps_warning_shown = 0U;
static uint8_t g_capability_source_state = 0U;
static struct
{
  uint16_t min_voltage_mv;
  uint16_t max_voltage_mv;
  uint16_t max_current_ma;
} g_pps_capabilities[POWER_MGR_MAX_PPS_APDOS] = {{0U, 0U, 0U}};
static uint8_t g_pps_capability_count = 0U;
static float g_calibration_target_voltage_v = 0.0f;
static float g_calibration_current_a = 0.0f;

static void PowerMgr_ResetNegotiation(void);
static uint8_t PowerMgr_GetPpsCurrentLimitMa(uint16_t voltage_mv,
                                             uint16_t *current_ma);

static void PowerMgr_SetChargeState(PowerMgr_ChargeState_t state)
{
  g_status.charge_state = state;
}

/*
 * PowerMgr_SetChargeOutput
 * ------------------------------------------------------------
 * 控制充电输出使能信号 CH_EN。
 *
 * 这个函数是安全策略的关键出口：
 *   - 当系统允许充电时，开 CH_EN；
 *   - 当出现过压、禁用、保护条件或电源协商失败时，关 CH_EN。
 *
 * 它还会重置一定的协商和负载跟踪状态，确保每次重新打开输出时，
 * 负载状态和历史电流数据从干净状态重新开始。这能避免因历史残留
 * 数据导致错误判断。
 */
static void PowerMgr_SetChargeOutput(uint8_t enabled)
{
  uint8_t next_enabled = (enabled != 0U) ? 1U : 0U;

  /*
   * 点焊逻辑必须拥有最高优先级：任何时候只要焊接互锁处于激活状态，
   * 充电输出都必须被强制关断，并且不能被后续状态机重新打开。
   * 这样可避免焊接脉冲与充电电流/均衡动作竞争，保证脉冲阶段的
   * 供电安全和后续恢复逻辑可在焊接结束后按正常流程继续。
   */
  if (g_weld_inhibit != 0U)
  {
    next_enabled = 0U;
  }

  if (g_charge_output_enabled == next_enabled)
  {
    if (next_enabled == 0U)
    {
      PowerMgr_ResetNegotiation();
    }
    return;
  }

  if (next_enabled == 0U)
  {
    PowerMgr_ResetNegotiation();
  }

  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin,
                    (next_enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_charge_output_enabled = next_enabled;
  g_negative_current_tracking = 0U;
  if (next_enabled != 0U)
  {
    g_last_current_sample_ms = g_status.measurement_tick_ms;
    g_last_ramp_ms = HAL_GetTick();
  }
}

static uint16_t PowerMgr_RoundPpsUp(float voltage_v)
{
  uint32_t voltage_mv = (uint32_t)(voltage_v * 1000.0f + 0.5f);
  uint16_t next_supported_mv = UINT16_MAX;
  uint8_t supported = 0U;
  uint8_t i = 0U;

  voltage_mv = ((voltage_mv + POWER_MGR_PPS_STEP_MV - 1U) /
                POWER_MGR_PPS_STEP_MV) * POWER_MGR_PPS_STEP_MV;
  if (voltage_mv < POWER_MGR_HARDWARE_MIN_MV)
  {
    voltage_mv = POWER_MGR_HARDWARE_MIN_MV;
  }
  if (voltage_mv > POWER_MGR_PPS_MAX_MV)
  {
    voltage_mv = POWER_MGR_PPS_MAX_MV;
  }

  for (i = 0U; i < g_pps_capability_count; i++)
  {
    if ((voltage_mv >= g_pps_capabilities[i].min_voltage_mv) &&
        (voltage_mv <= g_pps_capabilities[i].max_voltage_mv))
    {
      supported = 1U;
      break;
    }
    if ((g_pps_capabilities[i].min_voltage_mv > voltage_mv) &&
        (g_pps_capabilities[i].min_voltage_mv < next_supported_mv))
    {
      next_supported_mv = g_pps_capabilities[i].min_voltage_mv;
    }
  }
  if ((supported == 0U) && (next_supported_mv != UINT16_MAX))
  {
    voltage_mv = next_supported_mv;
  }
  return (uint16_t)voltage_mv;
}

static uint8_t PowerMgr_GetPpsCurrentLimitMa(uint16_t voltage_mv,
                                             uint16_t *current_ma)
{
  uint8_t i = 0U;
  uint16_t selected_current_ma = 0U;
  uint16_t maximum_current_ma = 0U;
  if (current_ma == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < g_pps_capability_count; i++)
  {
    uint16_t capability_current_ma =
        g_pps_capabilities[i].max_current_ma;
    if (capability_current_ma > maximum_current_ma)
    {
      maximum_current_ma = capability_current_ma;
    }
    if ((voltage_mv >= g_pps_capabilities[i].min_voltage_mv) &&
        (voltage_mv <= g_pps_capabilities[i].max_voltage_mv) &&
        (capability_current_ma > selected_current_ma))
    {
      selected_current_ma = capability_current_ma;
    }
  }

  if (voltage_mv == 0U)
  {
    selected_current_ma = maximum_current_ma;
  }
  *current_ma = selected_current_ma;
  return (selected_current_ma != 0U) ? 1U : 0U;
}

static uint8_t PowerMgr_GetCapacitorVoltage(float *voltage_v)
{
  return (uint8_t)((voltage_v != NULL) &&
                   (Control_GetTotalVoltage(voltage_v) != 0U));
}

/*
 * PowerMgr_RequestPps
 * ------------------------------------------------------------
 * 向 CH224A 发出 PPS 电压请求。
 *
 * 该函数会校验请求值是否合法，并要求其必须是步进 100 mV 的有效
 * 范围值。然后通过 CH224A_RequestPPSVoltage() 将目标值发给供电端，
 * 同时记录请求时间戳和 PPS 设置状态。
 *
 * 若请求失败，会清除协商和稳定判断状态，避免上层认为“已设定
 * 电压”但实际上供电端没接受该要求。
 */
static uint8_t PowerMgr_RequestPps(uint16_t voltage_mv)
{
  if ((voltage_mv < POWER_MGR_HARDWARE_MIN_MV) ||
      (voltage_mv > POWER_MGR_PPS_MAX_MV) ||
      ((voltage_mv % POWER_MGR_PPS_STEP_MV) != 0U))
  {
    return 0U;
  }

  if (CH224A_RequestPPSVoltage(ch224a_dev.i2c_addr, voltage_mv) != HAL_OK)
  {
    g_negotiation_started = 0U;
    g_pps_settle_tracking = 0U;
    return 0U;
  }

  g_status.pps_voltage_mv = voltage_mv;
  g_pps_request_ms = HAL_GetTick();
  g_pps_settle_tracking = 0U;
  g_pps_settle_sample_ms = 0U;
  return 1U;
}

/*
 * PowerMgr_ResetNegotiation
 * ------------------------------------------------------------
 * 复位 PPS 协商状态机。
 *
 * 这个函数会清空当前协商相关的中间量：
 *   - 初始电压；
 *   - 目标 PPS 电压；
 *   - 过冲/稳定跟踪标志；
 *   - 上次斜率判定时间。
 *
 * 这在输出被禁止、状态切换、错误恢复或重新开始协商时非常重要，
 * 因为若不重置，旧协商状态可能继续影响新的充电过程。
 */
static void PowerMgr_ResetNegotiation(void)
{
  g_negotiation_started = 0U;
  g_initial_voltage_mv = 0U;
  g_status.pps_voltage_mv = 0U;
  g_pps_settle_tracking = 0U;
  g_pps_settle_sample_ms = 0U;
  g_last_ramp_sample_ms = 0U;
  g_negative_current_tracking = 0U;
}

static uint8_t PowerMgr_ControlCurrent(uint32_t now, float capacitor_voltage_v,
                                       float target_voltage_v,
                                       float target_current_a)
{
  uint32_t sample_ms = g_status.measurement_tick_ms;
  uint16_t requested_mv = g_status.pps_voltage_mv;
  uint16_t next_mv = requested_mv;
  uint16_t minimum_safe_mv = PowerMgr_RoundPpsUp(capacitor_voltage_v);
  float current_a = g_status.input_current_a;
  uint8_t at_target = (capacitor_voltage_v >= target_voltage_v) ? 1U : 0U;

  if ((g_status.measurement_valid == 0U) ||
      ((uint32_t)(now - sample_ms) > POWER_MGR_MEASUREMENT_MAX_AGE_MS) ||
      (sample_ms == g_last_ramp_sample_ms) ||
      ((uint32_t)(now - g_last_ramp_ms) <
       POWER_MGR_PPS_RAMP_PERIOD_MS))
  {
    return 1U;
  }
  g_last_ramp_sample_ms = sample_ms;
  g_last_ramp_ms = now;

  if (current_a > target_current_a + POWER_MGR_CURRENT_DEADBAND_A)
  {
    if (requested_mv > minimum_safe_mv)
    {
      next_mv = (uint16_t)(requested_mv - POWER_MGR_PPS_STEP_MV);
      if (next_mv < minimum_safe_mv)
      {
        next_mv = minimum_safe_mv;
      }
    }
  }
  else if (at_target != 0U)
  {
    return 1U;
  }
  else if ((at_target == 0U) &&
           (current_a < target_current_a - POWER_MGR_CURRENT_DEADBAND_A))
  {
    uint16_t current_ramp_max_mv = POWER_MGR_PPS_MAX_MV;
    if (requested_mv < current_ramp_max_mv)
    {
      next_mv = (uint16_t)(requested_mv + POWER_MGR_PPS_STEP_MV);
      if (next_mv > current_ramp_max_mv)
      {
        next_mv = current_ramp_max_mv;
      }
    }
  }

  if (next_mv == requested_mv)
  {
    return 1U;
  }

  if (PowerMgr_RequestPps(next_mv) != 0U)
  {
    return 1U;
  }

  PowerMgr_SetChargeOutput(0U);
  PowerMgr_ResetNegotiation();
  PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
  return 0U;
}

/*
 * PowerMgr_Init
 * ------------------------------------------------------------
 * 初始化电源管理模块的全局状态和硬件接口。
 *
 * 关键动作：
 *   - 把 g_status 清零并设置 WAIT_SOURCE 状态；
 *   - 初始化时间戳，确保下一次采样和探测能按计划发生；
 *   - 关闭 CH_EN 输出，保证启动时不意外给负载通电；
 *   - 初始化 CH224A 模块。
 *
 * 这是系统在上电后的第一层安全保护：所有充电逻辑都不会在未
 * 确认输入状态前开始工作。
 */
void PowerMgr_Init(void)
{
  uint32_t now = HAL_GetTick();
  g_status = (PowerMgr_Status_t){0};
  g_status.charge_state = POWER_MGR_CHARGE_WAIT_SOURCE;
  g_last_sample_ms = now;
  g_last_detect_ms = now;
  g_last_ina_retry_ms = now - POWER_MGR_DEVICE_RETRY_PERIOD_MS;
  g_negative_since_ms = 0U;
  g_last_current_sample_ms = 0U;
  g_last_ramp_sample_ms = 0U;
  g_pps_request_ms = 0U;
  g_pps_settle_since_ms = 0U;
  g_pps_settle_sample_ms = 0U;
  g_last_ramp_ms = 0U;
  g_source_connected_since_ms = 0U;
  g_initial_voltage_mv = 0U;
  g_charge_output_enabled = 0U;
  g_weld_inhibit = 0U;
  g_calibration_inhibit = 0U;
  g_calibration_charge_active = 0U;
  g_negotiation_started = 0U;
  g_negative_current_tracking = 0U;
  g_pps_settle_tracking = 0U;
  g_cell_overvoltage_latched = 0U;
  g_cell_overvoltage_recovery_tracking = 0U;
  g_cell_overvoltage_recovery_since_ms = 0U;
  g_pps_warning_shown = 0U;
  g_capability_source_state = 0U;
  g_pps_capability_count = 0U;
  g_calibration_target_voltage_v = 0.0f;
  g_calibration_current_a = 0.0f;

  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, GPIO_PIN_RESET);
  CH224A_Init();
  g_status.ina226_connected = 0U;
}

static void PowerMgr_ParseSourceCapabilities(void)
{
  uint8_t protocol_status = ch224a_dev.protocol_status;
  uint8_t source_state = 0U;
  uint8_t data[CH224A_POWER_DATA_SIZE] = {0U};
  uint16_t header = 0U;
  uint16_t extended_header = 0U;
  uint16_t data_size = 0U;
  uint8_t pdo_count = 0U;
  uint8_t pdo_offset = 0U;
  uint8_t i = 0U;
  uint8_t is_extended = 0U;

  if ((protocol_status & 0x10U) != 0U)
  {
    source_state = 2U;
  }
  else if ((protocol_status & 0x08U) != 0U)
  {
    source_state = 1U;
  }

  if (source_state == 0U)
  {
    g_capability_source_state = 0U;
    g_pps_capability_count = 0U;
    return;
  }
  if (g_capability_source_state == source_state)
  {
    return;
  }
  g_pps_capability_count = 0U;
  if (CH224A_ReadPowerData(data) != HAL_OK)
  {
    return;
  }

  header = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
  is_extended = (uint8_t)((header & 0x8000U) != 0U);
  if ((header == 0U) ||
      ((header & 0x1FU) != (is_extended != 0U ? 0x11U : 0x01U)) ||
      (is_extended != (uint8_t)(source_state == 2U)))
  {
    g_capability_source_state = source_state;
    return;
  }

  if (is_extended != 0U)
  {
    extended_header = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
    data_size = extended_header & 0x01FFU;
    if (((data_size % 4U) != 0U) ||
        (data_size > (CH224A_POWER_DATA_SIZE - 4U)))
    {
      g_capability_source_state = source_state;
      return;
    }
    pdo_count = (uint8_t)(data_size / 4U);
    pdo_offset = 4U;
  }
  else
  {
    pdo_count = (uint8_t)((header >> 12U) & 0x07U);
    pdo_offset = 2U;
  }

  for (i = 0U; i < pdo_count; i++)
  {
    uint8_t offset = (uint8_t)(pdo_offset + (i * 4U));
    uint32_t pdo = (uint32_t)data[offset] |
                   ((uint32_t)data[offset + 1U] << 8U) |
                   ((uint32_t)data[offset + 2U] << 16U) |
                   ((uint32_t)data[offset + 3U] << 24U);
    if ((uint8_t)(pdo >> 30U) == 3U)
    {
      uint8_t apdo_type = (uint8_t)((pdo >> 28U) & 0x03U);
      if (apdo_type == 0U)
      {
        if (g_pps_capability_count < POWER_MGR_MAX_PPS_APDOS)
        {
          uint16_t min_voltage_mv =
              (uint16_t)(((pdo >> 8U) & 0x00FFU) * 100U);
          uint16_t max_voltage_mv =
              (uint16_t)(((pdo >> 17U) & 0x00FFU) * 100U);
          uint16_t current_ma = (uint16_t)((pdo & 0x007FU) * 50U);
          g_pps_capabilities[g_pps_capability_count].min_voltage_mv =
              min_voltage_mv;
          g_pps_capabilities[g_pps_capability_count].max_voltage_mv =
              max_voltage_mv;
          g_pps_capabilities[g_pps_capability_count].max_current_ma =
              current_ma;
          g_pps_capability_count++;
        }
      }
    }
  }
  if ((g_pps_capability_count == 0U) &&
      (g_pps_warning_shown == 0U))
  {
    TftUi_NotifyMessage("请更换PPS协议电源", 3000U);
    g_pps_warning_shown = 1U;
  }
  g_capability_source_state = source_state;
}

/*
 * PowerMgr_Task
 * ------------------------------------------------------------
 * 周期性扫描外部输入源和 INA226 设备状态。
 *
 * 它的职责包括：
 *   - 以一定周期探测 CH224A 是否存在；
 *   - 从 PD Source Capabilities 中读取 PPS 电流能力；
 *   - 以间隔方式初始化或重试 INA226；
 *   - 若成功读取电压/电流/功率，则更新 g_status.measurement_valid。
 *   - PD/EPR 握手成功后解析 PPS 电流能力。
 *
 * 这个任务不是控制充电本身，而是给整个电源管理系统提供可靠的
 * 输入状态基础；之后的 ChargeTask 会根据这些状态决定是否开启或
 * 停止充电。
 */
void PowerMgr_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t pps_current_limit_ma = 0U;
  uint32_t detect_period_ms =
      (g_status.ch224a_connected != 0U) ?
      POWER_MGR_DETECT_PERIOD_MS : POWER_MGR_DISCONNECTED_RETRY_MS;

  if ((uint32_t)(now - g_last_detect_ms) >= detect_period_ms)
  {
    uint8_t was_connected = g_status.ch224a_connected;
    g_last_detect_ms = now;
    (void)CH224A_DetectDevice();
    g_status.ch224a_connected = CH224A_IsConnected();
    if (g_status.ch224a_connected == 0U)
    {
      g_source_connected_since_ms = 0U;
      g_capability_source_state = 0U;
      g_pps_capability_count = 0U;
      g_pps_warning_shown = 0U;
    }
    else
    {
      if (was_connected == 0U)
      {
        g_source_connected_since_ms = now;
      }
      PowerMgr_ParseSourceCapabilities();
    }
  }
  if ((g_status.ch224a_connected != 0U) &&
      (g_pps_capability_count == 0U) &&
      (g_pps_warning_shown == 0U) &&
      ((ch224a_dev.protocol_status & 0x18U) == 0U) &&
      ((uint32_t)(now - g_source_connected_since_ms) >=
       POWER_MGR_PPS_DETECTION_TIMEOUT_MS))
  {
    TftUi_NotifyMessage("请更换PPS协议电源", 3000U);
    g_pps_warning_shown = 1U;
  }
  g_status.source_current_valid =
      PowerMgr_GetPpsCurrentLimitMa(g_status.pps_voltage_mv,
                                    &pps_current_limit_ma);
  g_status.source_max_current_a =
      (float)pps_current_limit_ma / 1000.0f;

  if ((g_status.ina226_connected == 0U) &&
      ((uint32_t)(now - g_last_ina_retry_ms) >=
       POWER_MGR_DEVICE_RETRY_PERIOD_MS))
  {
    g_last_ina_retry_ms = now;
    INA226_Init();
    g_status.ina226_connected = INA226_IsConnected();
  }

  if ((uint32_t)(now - g_last_sample_ms) < POWER_MGR_SAMPLE_PERIOD_MS)
  {
    return;
  }
  g_last_sample_ms = now;
  g_status.measurement_valid = 0U;

  if (g_status.ina226_connected != 0U)
  {
    INA226_Measurements_t measurements = {0};
    if (INA226_ReadMeasurements(&measurements) == HAL_OK)
    {
      g_status.input_voltage_v = measurements.bus_voltage_v;
      g_status.input_current_a = measurements.current_a;
      g_status.input_power_w = measurements.power_w;
      g_status.measurement_valid = 1U;
      g_status.measurement_tick_ms = now;
    }
    else
    {
      g_status.ina226_connected = 0U;
    }
  }

  g_status.status_mask = 0U;
  if (g_status.ch224a_connected != 0U)
  {
    g_status.status_mask |= POWER_MGR_STATUS_CH224A_PRESENT;
  }
  if (g_status.ina226_connected != 0U)
  {
    g_status.status_mask |= POWER_MGR_STATUS_INA226_PRESENT;
  }
  if (g_status.measurement_valid != 0U)
  {
    g_status.status_mask |= POWER_MGR_STATUS_INPUT_VALID;
  }

}

/*
 * PowerMgr_ChargeTask
 * ------------------------------------------------------------
 * 真正决定是否允许充电以及充电状态切换的核心任务。
 *
 * 它在每次循环中会：
 *   - 获取当前电容组电压和过压状态；
 *   - 读取设定的目标电压、电流、是否允许充电；
 *   - 检查配置禁用、焊接互锁、过压和校准互锁；
 *   - 如果所有条件满足，则让充电状态进入 WAIT_SOURCE/NEGOTIATING/CC/CV 等状态机流程。
 *
 * 这个函数是整个电源控制安全逻辑的集中枢纽：它不会直接做硬件
 * 操作，而是把判断结果交给 SetChargeOutput() 和状态切换逻辑，从而
 * 降低逻辑分散和避免安全策略遗漏。
 */
void PowerMgr_ChargeTask(void)
{
  uint32_t now = HAL_GetTick();
  float capacitor_voltage_v = 0.0f;
  float target_voltage_v = 0.0f;
  float configured_capacitor_limit_v = 0.0f;
  float target_current_a = 0.0f;
  float overvoltage_cell_v = 0.0f;
  float overvoltage_limit_v = 0.0f;
  uint8_t capacitor_overvoltage = 0U;
  uint8_t capacitor_voltage_valid =
      PowerMgr_GetCapacitorVoltage(&capacitor_voltage_v);
  uint8_t overvoltage_cell = 0U;
  uint8_t recovery_safe = 0U;
  uint8_t cell_index = 0U;
  uint8_t cell_overvoltage = Control_GetOvervoltageCell(
      &overvoltage_cell, &overvoltage_cell_v, &overvoltage_limit_v);
  uint8_t calibration_charge_active = g_calibration_charge_active;
  uint8_t charge_requested = 0U;
  TftUiSettings_t settings = {0};

  TftUi_GetSettings(&settings);
  target_voltage_v = (calibration_charge_active != 0U) ?
                     g_calibration_target_voltage_v :
                     settings.cell_voltage * (float)settings.capacitor_series;
  configured_capacitor_limit_v =
      settings.cell_voltage * (float)settings.capacitor_series;
  capacitor_overvoltage =
      (uint8_t)((capacitor_voltage_valid != 0U) &&
                (configured_capacitor_limit_v > 0.0f) &&
                (capacitor_voltage_v > configured_capacitor_limit_v));
  target_current_a = (calibration_charge_active != 0U) ?
                     g_calibration_current_a : settings.charge_current_a;
  if (g_status.source_current_valid != 0U)
  {
    float source_current_limit_a =
        g_status.source_max_current_a * POWER_MGR_SOURCE_CURRENT_MARGIN;
    if (target_current_a > source_current_limit_a)
    {
      target_current_a = source_current_limit_a;
    }
  }
  charge_requested =
      (uint8_t)((calibration_charge_active != 0U) ||
                (settings.charge_enable != 0U));

  if ((cell_overvoltage != 0U) || (capacitor_overvoltage != 0U))
  {
    g_cell_overvoltage_latched = 1U;
    g_cell_overvoltage_recovery_tracking = 0U;
  }

  if (g_cell_overvoltage_latched != 0U)
  {
    if ((capacitor_voltage_valid != 0U) &&
        (settings.capacitor_series > 0U) &&
        (settings.capacitor_series <= 4U) &&
        (settings.cell_voltage > POWER_MGR_OVERVOLTAGE_RECOVERY_HYST_V) &&
        (configured_capacitor_limit_v >
         POWER_MGR_OVERVOLTAGE_RECOVERY_HYST_V) &&
        (capacitor_voltage_v <= configured_capacitor_limit_v -
                                POWER_MGR_OVERVOLTAGE_RECOVERY_HYST_V))
    {
      recovery_safe = 1U;
      for (cell_index = 0U; cell_index < settings.capacitor_series;
           cell_index++)
      {
        if (Control_GetCellVoltage(cell_index) >
            settings.cell_voltage - POWER_MGR_OVERVOLTAGE_RECOVERY_HYST_V)
        {
          recovery_safe = 0U;
          break;
        }
      }
    }

    if (recovery_safe != 0U)
    {
      if (g_cell_overvoltage_recovery_tracking == 0U)
      {
        g_cell_overvoltage_recovery_since_ms = now;
        g_cell_overvoltage_recovery_tracking = 1U;
      }
      else if ((uint32_t)(now - g_cell_overvoltage_recovery_since_ms) >=
               POWER_MGR_OVERVOLTAGE_RECOVERY_MS)
      {
        g_cell_overvoltage_latched = 0U;
        g_cell_overvoltage_recovery_tracking = 0U;
      }
    }
    else
    {
      g_cell_overvoltage_recovery_tracking = 0U;
    }
  }

  if (g_cell_overvoltage_latched != 0U)
  {
    if (g_charge_output_enabled != 0U)
    {
      PowerMgr_SetChargeOutput(0U);
    }
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
    return;
  }

  /*
   * 充电任务的最终判定顺序里，点焊互锁必须排在所有正常充电条件之前。
   * 这是因为焊接脉冲输出期间，电容放电和器件负载会出现极大瞬态，
   * 若继续允许充电，则会造成焊接功率不稳定和输出控制竞争。
   * 因此只要 g_weld_inhibit 被置位，无论其他条件是否允许，都会被
   * 直接切断 CH_EN，并进入 DISABLED 状态，等焊接结束后再恢复。
   */
  if ((cell_overvoltage != 0U) ||
      (charge_requested == 0U) ||
      (target_current_a <= 0.0f) ||
      (g_weld_inhibit != 0U) ||
      !(configured_capacitor_limit_v > 0.0f) ||
      ((g_calibration_inhibit != 0U) &&
       (calibration_charge_active == 0U)))
  {
    PowerMgr_SetChargeOutput(0U);
    PowerMgr_ResetNegotiation();
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_DISABLED);
    return;
  }

  if (capacitor_voltage_valid == 0U)
  {
    PowerMgr_SetChargeOutput(0U);
    PowerMgr_ResetNegotiation();
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
    return;
  }

  if (g_status.charge_state == POWER_MGR_CHARGE_COMPLETE)
  {
    if (capacitor_voltage_v >=
        target_voltage_v - POWER_MGR_COMPLETE_RECHARGE_HYST_V)
    {
      return;
    }
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
  }
  if (capacitor_voltage_v >= target_voltage_v)
  {
    PowerMgr_SetChargeOutput(0U);
    PowerMgr_ResetNegotiation();
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_COMPLETE);
    return;
  }

  if ((g_charge_output_enabled != 0U) &&
      (g_status.source_current_valid == 0U))
  {
    PowerMgr_SetChargeOutput(0U);
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
    return;
  }

  if (g_charge_output_enabled != 0U)
  {
    if ((g_status.measurement_valid == 0U) ||
        ((uint32_t)(now - g_status.measurement_tick_ms) >
         POWER_MGR_MEASUREMENT_MAX_AGE_MS))
    {
      PowerMgr_SetChargeOutput(0U);
      PowerMgr_ResetNegotiation();
      PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
      return;
    }

    if (g_status.measurement_tick_ms != g_last_current_sample_ms)
    {
      uint32_t sample_ms = g_status.measurement_tick_ms;
      g_last_current_sample_ms = sample_ms;
      if (g_status.input_current_a >=
          g_status.source_max_current_a *
          POWER_MGR_SOURCE_OVERCURRENT_MARGIN)
      {
        PowerMgr_SetChargeOutput(0U);
        PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
        return;
      }
      if (g_status.input_current_a < 0.0f)
      {
        if (g_negative_current_tracking == 0U)
        {
          g_negative_since_ms = sample_ms;
          g_negative_current_tracking = 1U;
        }
        else if ((uint32_t)(sample_ms - g_negative_since_ms) >=
                 POWER_MGR_NEGATIVE_CURRENT_MS)
        {
          PowerMgr_SetChargeOutput(0U);
          PowerMgr_ResetNegotiation();
          PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
          return;
        }
      }
      else
      {
        g_negative_current_tracking = 0U;
      }
    }

    if (g_status.ch224a_connected != 0U)
    {
      if (PowerMgr_ControlCurrent(now, capacitor_voltage_v,
                                  target_voltage_v, target_current_a) == 0U)
      {
        return;
      }
    }
    PowerMgr_SetChargeState((capacitor_voltage_v >= target_voltage_v) ?
                            POWER_MGR_CHARGE_CV : POWER_MGR_CHARGE_CC);
    return;
  }

  if (g_status.ch224a_connected == 0U)
  {
    PowerMgr_ResetNegotiation();
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
    return;
  }

  if ((g_status.measurement_valid == 0U) ||
      ((uint32_t)(now - g_status.measurement_tick_ms) >
       POWER_MGR_MEASUREMENT_MAX_AGE_MS))
  {
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
    return;
  }

  if (g_negotiation_started == 0U)
  {
    float initial_voltage_v =
        (capacitor_voltage_v >= target_voltage_v) ?
        target_voltage_v : capacitor_voltage_v;
    g_initial_voltage_mv = PowerMgr_RoundPpsUp(initial_voltage_v);
    if (PowerMgr_RequestPps(g_initial_voltage_mv) == 0U)
    {
      PowerMgr_SetChargeState(POWER_MGR_CHARGE_WAIT_SOURCE);
      return;
    }
    g_negotiation_started = 1U;
  }

  if (((uint32_t)(g_status.measurement_tick_ms - g_pps_request_ms) == 0U) ||
      ((uint32_t)(g_status.measurement_tick_ms - g_pps_request_ms) >=
       0x80000000UL))
  {
    g_pps_settle_tracking = 0U;
    g_pps_settle_sample_ms = 0U;
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_NEGOTIATING);
    return;
  }

  if (g_status.measurement_tick_ms == g_pps_settle_sample_ms)
  {
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_NEGOTIATING);
    return;
  }
  g_pps_settle_sample_ms = g_status.measurement_tick_ms;

  if (g_pps_settle_tracking == 0U)
  {
    g_pps_settle_since_ms = g_status.measurement_tick_ms;
    g_pps_settle_tracking = 1U;
  }
  else if ((uint32_t)(g_status.measurement_tick_ms - g_pps_settle_since_ms) <
           POWER_MGR_PPS_SETTLE_MS)
  {
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_NEGOTIATING);
    return;
  }

  if (g_status.source_current_valid == 0U)
  {
    PowerMgr_SetChargeState(POWER_MGR_CHARGE_NEGOTIATING);
    return;
  }

  PowerMgr_SetChargeOutput(1U);
  PowerMgr_SetChargeState((capacitor_voltage_v >= target_voltage_v) ?
                          POWER_MGR_CHARGE_CV : POWER_MGR_CHARGE_CC);
}

uint8_t PowerMgr_IsInputValid(void)
{
  return g_status.measurement_valid;
}

float PowerMgr_GetInputVoltageV(void)
{
  return g_status.input_voltage_v;
}

float PowerMgr_GetInputCurrentA(void)
{
  return g_status.input_current_a;
}

uint8_t PowerMgr_GetStatusMask(void)
{
  return g_status.status_mask;
}

/*
 * PowerMgr_GetStatus
 * ------------------------------------------------------------
 * 把当前电源管理状态复制给调用者。
 *
 * 这通常由 UI 或控制逻辑使用，用于读取：
 *   - 输入电压；
 *   - 输入电流；
 *   - 是否探测到 CH224A/INA226；
 *   - 当前处于哪一个充电状态；
 *   - 当前是否有有效的电源测量。
 *
 * 通过此接口，上层不需要直接访问静态全局变量，能更稳妥地读取状态。
 */
void PowerMgr_GetStatus(PowerMgr_Status_t *status)
{
  if (status != NULL)
  {
    *status = g_status;
  }
}

/*
 * PowerMgr_StartCalibrationCharge
 * ------------------------------------------------------------
 * 启动校准期间的专用充电任务。
 *
 * 该功能用于要求系统在一个受控的目标电压和目标电流下进行充电，
 * 通常配合容量校准和压降校准使用。它会记录：
 *   - 目标电压；
 *   - 目标电流；
 *   - 当前是否处于校准模式；
 * 同时将系统的充电状态切到需要协商的路径，以供校准任务使用。
 */
uint8_t PowerMgr_StartCalibrationCharge(float target_voltage_v, float current_a)
{
  TftUiSettings_t settings = {0};
  float configured_capacitor_limit_v = 0.0f;

  TftUi_GetSettings(&settings);
  configured_capacitor_limit_v =
      settings.cell_voltage * (float)settings.capacitor_series;
  if (!(target_voltage_v > 0.0f) ||
      !(configured_capacitor_limit_v > 0.0f) ||
      (target_voltage_v > configured_capacitor_limit_v) ||
      !(current_a > 0.0f) || !(current_a <= 8.0f) ||
      (g_calibration_inhibit == 0U) ||
      (g_weld_inhibit != 0U) ||
      (CH224A_IsConnected() == 0U))
  {
    return 0U;
  }

  PowerMgr_SetChargeOutput(0U);
  g_calibration_target_voltage_v = target_voltage_v;
  g_calibration_current_a = current_a;
  g_calibration_charge_active = 1U;
  PowerMgr_ResetNegotiation();
  return 1U;
}

void PowerMgr_StopCalibrationCharge(void)
{
  g_calibration_charge_active = 0U;
  g_calibration_target_voltage_v = 0.0f;
  g_calibration_current_a = 0.0f;
  PowerMgr_SetChargeOutput(0U);
  PowerMgr_ResetNegotiation();
}

/*
 * PowerMgr_SetCalibrationMode
 * ------------------------------------------------------------
 * 设置是否启用校准模式的全局开关。
 *
 * 在校准过程中，系统不能照常接受用户配置的充电允许条件，需让
 * 电源管理模块临时放开或限制某些功能。因此，此函数会在合适时机
 * 打开/关闭 g_calibration_inhibit 等状态，以确保校准期间不会被
 * 其他保护逻辑误判。
 */
void PowerMgr_SetCalibrationMode(uint8_t active)
{
  g_calibration_inhibit = (active != 0U) ? 1U : 0U;
  if (g_calibration_inhibit != 0U)
  {
    PowerMgr_StopCalibrationCharge();
  }
}

/*
 * PowerMgr_SetWeldInhibit
 * ------------------------------------------------------------
 * 设置点焊互锁状态。
 *
 * 当系统正在进行焊接或电容发生异常状态时，必须禁止继续充电，
 * 否则可能导致焊接过程和充电状态互相干扰。这个函数就是上层逻辑
 * 对“充电应被暂时锁定”这一条件的统一表达。
 */
void PowerMgr_SetWeldInhibit(uint8_t active)
{
  g_weld_inhibit = (active != 0U) ? 1U : 0U;
  if (g_weld_inhibit != 0U)
  {
    PowerMgr_SetChargeOutput(0U);
    PowerMgr_ResetNegotiation();
  }
}
