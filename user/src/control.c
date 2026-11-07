#include "control.h"
#include <stdio.h>
#include <string.h>
#include "buzzer.h"
#include "adc.h"
#include "tim.h"
#include "ina226.h"
#include "power_mgr.h"

/*
 * control.c
 * ------------------------------------------------------------
 * 控制核心实现：处理电容电压采样、均衡、点焊时序和保护逻辑。
 *
 * 这个文件是整个系统的动作中心，负责把用户设置转换为实际硬件动作。
 * 例如：
 *   - 根据电压水平判断是否触发均衡；
 *   - 根据工艺参数控制点焊脉冲；
 *   - 根据 ADC 和 INA226 测量数据做安全熔断；
 *   - 处理触摸和校准流程。
 *
 * 它会在主循环中周期扫描，以保证所有设备状态都在最短时间内反映。
 */

#define CONTROL_SAMPLE_PERIOD_MS       50U
#define ADC_VREF_CAL_VDDA_MV          3300.0f

/* 通过内部参考电压采样估算当前 ADC 的 VDDA。 */
static float Control_GetActualVrefMv(void)
{
  /* STM32F1出厂内部参考电压(VREFINT)校准值存储地址 (1.2V标定, 3.3V VDD下) */
  uint16_t vrefint_cal = *((volatile uint16_t *)0x1FFFF7BAU);
  uint32_t adc_vrefint_raw = 0U;
  ADC_ChannelConfTypeDef sConfig = {0};

  if (vrefint_cal == 0U || vrefint_cal == 0xFFFFU)
  {
    return 3300.0f; /* 若没有出厂校准值，回退到默认3.3V */
  }

  /* 临时配置ADC1通道17为内部参考电压通道进行单次采样 */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 3300.0f;
  }

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
  {
    adc_vrefint_raw = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  /* 恢复原本的通道1配置以供DMA循环采样使用 */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  if (adc_vrefint_raw == 0U)
  {
    return 3300.0f;
  }

  /* 根据VREFINT实时计算当前的实际VDDA(单位: mV) */
  {
    float actual_vdda_mv = (ADC_VREF_CAL_VDDA_MV * (float)vrefint_cal) / (float)adc_vrefint_raw;
    /* 限制在合理的3.0V ~ 3.6V范围内，防止异常干扰 */
    if (actual_vdda_mv < 3000.0f || actual_vdda_mv > 3600.0f)
    {
      return 3300.0f;
    }
    return actual_vdda_mv;
  }
}

#define ADC_MAX_COUNTS                4095.0f
#define ADC_CELL_VOLTAGE_GAIN            1.0f
#define CONTROL_BALANCE_SETTLE_MS          5U
#define CONTROL_SAFETY_RECOVERY_SAMPLE_COUNT 3U
#define ADC_AVERAGE_GROUP_COUNT          16U
#define ADC_AVERAGE_TIMEOUT_MS           20U

typedef enum
{
  CONTROL_ADC_SAMPLE_PENDING = 0U,
  CONTROL_ADC_SAMPLE_READY = 1U,
  CONTROL_ADC_SAMPLE_FAILED = 2U
} ControlAdcSampleResult_t;
#define CURRENT_FAULT_LIMIT_A           18.0f
#define WELD_PWM_COMPARE               800U
#define WELD_COOLDOWN_MS               300U
#define WELD_CONTACT_TEST_MS             5U
#define WELD_VOLTAGE_MIN_RATIO         0.78f
#define WELD_REFERENCE_VOLTAGE_RATIO   0.90f
#define WELD_MIN_COMPENSATION           0.75f
#define WELD_MAX_COMPENSATION           1.50f
#define CALIBRATION_SAMPLE_MS             100U
#define CALIBRATION_CURRENT_A             1.5f
#define CALIBRATION_VSTART_RATIO          0.65f
#define CALIBRATION_VEND_RATIO            0.83f
#define CALIBRATION_VDROP_MIN_RATIO     0.85f
#define CALIBRATION_VDROP_REFERENCE_RATIO 0.90f
#define CALIBRATION_VDROP_MAX_RATIO     0.95f
#define CALIBRATION_DROP_PULSE_COUNT       5U
#define CALIBRATION_DROP_PULSE_MS        500U
#define CALIBRATION_DROP_MIN_V          0.005f
#define CALIBRATION_DROP_MAX_V          3.000f

static uint32_t Control_RoundVoltageMillivolts(float voltage_v)
{
  return (uint32_t)(voltage_v * 1000.0f + 0.5f);
}

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

typedef enum
{
  CONTROL_CALIBRATION_NONE = 0,
  CONTROL_CALIBRATION_CAPACITOR_BLEED,
  CONTROL_CALIBRATION_CAPACITOR_CHARGE,
  CONTROL_CALIBRATION_VDROP_PULSE
} ControlCalibrationPhase_t;

typedef enum
{
  CONTROL_WELD_IDLE = 0,
  CONTROL_WELD_PULSE_1,
  CONTROL_WELD_INTERVAL,
  CONTROL_WELD_PULSE_2
} ControlWeldPhase_t;

static GPIO_TypeDef * const g_balance_ports[4] =
{
  C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port, C4_GPIO_Port
};
static const uint16_t g_balance_pins[4] =
{
  C1_Pin, C2_Pin, C3_Pin, C4_Pin
};

static uint32_t g_last_control_scan_ms = 0U;
static uint32_t g_last_capacity_integral_ms = 0U;
static uint32_t g_calibration_phase_tick_ms = 0U;
static uint32_t g_weld_phase_tick_ms = 0U;
static volatile uint32_t g_weld_cooldown_until_ms = 0U;
static uint32_t g_touch_trigger_ms = 0U;
static uint32_t g_total_weld_count = 0U;
static uint32_t g_session_weld_count = 0U;
static volatile uint16_t g_adc_dma_buffer[4U] = {0U};
static volatile uint16_t g_adc_average[4U] = {0U};
static volatile uint32_t g_adc_average_sum[4U] = {0U};
static volatile uint8_t g_adc_average_group_count = 0U;
static volatile uint8_t g_adc_average_collecting = 0U;
static volatile uint8_t g_adc_average_ready = 0U;
static uint32_t g_adc_average_start_ms = 0U;
static uint8_t g_adc_balance_settling = 0U;
static uint32_t g_adc_balance_settle_start_ms = 0U;
static float g_cell_voltage[4U] = {0.0f};
static float g_current_a = 0.0f;
static float g_capacity_charge_c = 0.0f;
static float g_capacity_start_voltage_v = 0.0f;
static float g_capacity_start_threshold_v = 0.0f;
static float g_capacity_end_threshold_v = 0.0f;
static float g_weld_start_voltage_v = 0.0f;
static float g_weld_compensation = 1.0f;
static uint16_t g_weld_pulse1_ms = 0U;
static uint16_t g_weld_pulse2_ms = 0U;
static uint16_t g_weld_interval_ms = 0U;
static uint8_t g_fault_latched = 0U;
static uint8_t g_safety_recovery_sample_count = 0U;
static volatile uint8_t g_touch_pending = 0U;
static volatile uint8_t g_touch_down = 0U;
static uint8_t g_cell_measurements_valid = 0U;
static uint8_t g_settings_recheck_pending = 0U;
static uint8_t g_adc_dma_started = 0U;
static uint8_t g_capacity_integral_started = 0U;
static uint8_t g_weld_sequence_counted = 0U;
static uint8_t g_calibration_drop_count = 0U;
static float g_total_voltage_v = 0.0f;
static float g_calibration_drop_samples[CALIBRATION_DROP_PULSE_COUNT] = {0.0f};
static volatile ControlCalibrationPhase_t g_calibration_phase = CONTROL_CALIBRATION_NONE;
static volatile ControlWeldPhase_t g_weld_phase = CONTROL_WELD_IDLE;
static TftUiCalibrationData_t g_calibration_data = {0};
static TftUiWorkState_t g_calibration_previous_work_state = TFT_UI_STATE_READY;

/* 提前声明常规输出任务，供前置的点焊逻辑调用。 */
static void Control_NormalOutputTask(void);

/* 读取并校验当前设置的串联电容数量。 */
static uint8_t Control_GetCellCount(void)
{
  TftUiSettings_t settings = {0};
  TftUi_GetSettings(&settings);
  return ((settings.capacitor_series >= 1U) && (settings.capacitor_series <= 4U)) ?
         settings.capacitor_series : 0U;
}

/* 根据单体耐压和串数计算电容组额定电压。 */
static float Control_GetRatedTotalVoltage(void)
{
  TftUiSettings_t settings = {0};
  TftUi_GetSettings(&settings);
  return settings.cell_voltage * (float)settings.capacitor_series;
}

/* 关闭所有单体均衡负载输出。 */
static void Control_DisableBalanceOutputs(void)
{
  uint8_t i = 0U;
  for (i = 0U; i < 4U; i++)
  {
    HAL_GPIO_WritePin(g_balance_ports[i], g_balance_pins[i], GPIO_PIN_RESET);
  }
}

/*
 * Control_PrepareVoltageSampling
 * ------------------------------------------------------------
 * 以非阻塞方式启动或检查一轮 ADC 平均采样。
 *
 * 返回 PENDING 时，调用方必须在后续任务周期再次调用；只有 DMA
 * 回调完成足够数量的样本后才返回 READY。超过限定时间则清理采样
 * 状态并返回 FAILED，防止 ADC/DMA 异常时一直处于等待状态。
 */
static ControlAdcSampleResult_t Control_PrepareVoltageSampling(void)
{
  uint32_t primask = 0U;
  uint32_t now = HAL_GetTick();
  uint8_t i = 0U;

  /* 关闭均衡输出后不再做长停顿；让 ADC 的 DMA 收集在后台继续进行。 */
  Control_DisableBalanceOutputs();

  primask = __get_PRIMASK();
  __disable_irq();
  if (g_adc_average_ready != 0U)
  {
    g_adc_average_ready = 0U;
    __set_PRIMASK(primask);
    return CONTROL_ADC_SAMPLE_READY;
  }

  if (g_adc_average_collecting != 0U)
  {
    if ((uint32_t)(now - g_adc_average_start_ms) >= ADC_AVERAGE_TIMEOUT_MS)
    {
      g_adc_average_collecting = 0U;
      __set_PRIMASK(primask);
      return CONTROL_ADC_SAMPLE_FAILED;
    }
    __set_PRIMASK(primask);
    return CONTROL_ADC_SAMPLE_PENDING;
  }

  if (g_adc_balance_settling != 0U)
  {
    if ((uint32_t)(now - g_adc_balance_settle_start_ms) <
        CONTROL_BALANCE_SETTLE_MS)
    {
      __set_PRIMASK(primask);
      return CONTROL_ADC_SAMPLE_PENDING;
    }
    g_adc_balance_settling = 0U;
  }
  else
  {
    g_adc_balance_settling = 1U;
    g_adc_balance_settle_start_ms = now;
    __set_PRIMASK(primask);
    return CONTROL_ADC_SAMPLE_PENDING;
  }

  for (i = 0U; i < 4U; i++)
  {
    g_adc_average_sum[i] = 0U;
  }
  g_adc_average_group_count = 0U;
  g_adc_average_ready = 0U;
  g_adc_average_collecting = 1U;
  g_adc_average_start_ms = now;
  __set_PRIMASK(primask);

  return CONTROL_ADC_SAMPLE_PENDING;
}

/* 在临界区内复制一组 ADC 平均采样值。 */
static void Control_GetAdcAverageSnapshot(uint16_t samples[4U])
{
  uint32_t primask = __get_PRIMASK();
  uint8_t i = 0U;

  __disable_irq();
  for (i = 0U; i < 4U; i++)
  {
    samples[i] = g_adc_average[i];
  }
  __set_PRIMASK(primask);
}

/* 停止点焊 PWM 并清除定时器单脉冲模式。 */
static void Control_StopPulseOutput(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  htim1.Instance->CR1 &= ~TIM_CR1_OPM;
}

/* 使用最新 ADC 平均值计算电容组总电压。 */
static float Control_ReadTotalVoltage(void)
{
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();
  uint16_t samples[4U] = {0U};
  float total = 0.0f;
  float dynamic_vref_mv = Control_GetActualVrefMv();

  if ((g_adc_dma_started == 0U) || (count == 0U))
  {
    return 0.0f;
  }
  Control_GetAdcAverageSnapshot(samples);
  for (i = 0U; i < count; i++)
  {
    float pin_voltage = ((float)samples[i] * dynamic_vref_mv) / ADC_MAX_COUNTS;
    total += (pin_voltage * ADC_CELL_VOLTAGE_GAIN) / 1000.0f;
  }
  return total;
}

/* 输出受限时长的测试脉冲并检测定时器是否正常结束。 */
static uint8_t Control_RunTestPulse(void)
{
  uint32_t start_tick = HAL_GetTick();

  Control_StopPulseOutput();
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, WELD_PWM_COMPARE);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  /* 接触测试使用单脉冲模式，并限制等待时间，防止定时器异常卡住。 */
  htim1.Instance->CR1 |= TIM_CR1_OPM;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Control_StopPulseOutput();
    return 0U;
  }

  while ((htim1.Instance->CR1 & TIM_CR1_CEN) != 0U)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) > WELD_CONTACT_TEST_MS)
    {
      Control_StopPulseOutput();
      return 0U;
    }
  }
  Control_StopPulseOutput();
  return 1U;
}

/* 将控制模块的工作状态同步到界面。 */
static void Control_SetWorkState(TftUiWorkState_t state)
{
  TftUi_SetWorkState(state);
}

/* 检查过流及反向电流保护。 */
static uint8_t Control_CurrentSafetyFaultActive(void)
{
  return (uint8_t)((g_current_a > CURRENT_FAULT_LIMIT_A) || (g_current_a < -5.0f));
}

/* 检查单体过压及电流越界等安全故障。 */
static uint8_t Control_SafetyFaultActive(void)
{
  TftUiSettings_t settings = {0};
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();
  TftUi_GetSettings(&settings);

  for (i = 0U; i < count; i++)
  {
    if (g_cell_voltage[i] > (settings.cell_voltage + 0.5f))
    {
      return 1U;
    }
  }
  return Control_CurrentSafetyFaultActive();
}

/* 统一停止校准相关输出并恢复校准前的工作状态。 */
static void Control_CalibrationFinish(const char *status)
{
  PowerMgr_StopCalibrationCharge();
  PowerMgr_SetCalibrationMode(0U);
  Control_DisableBalanceOutputs();
  Control_StopPulseOutput();
  g_calibration_phase = CONTROL_CALIBRATION_NONE;
  TftUi_SetCalibrationStatus(status);
  TftUi_SetCalibrationActive(0U);
  Control_SetWorkState(g_calibration_previous_work_state);
}

/* 放电到起始电压后充电，并通过库仑积分恢复电容量校准。 */
static void Control_CapacityCalibrationTask(uint32_t now)
{
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();
  uint8_t cell_above_start = 0U;
  float total_voltage = Control_ReadTotalVoltage();
  TftUiSettings_t settings = {0};
  PowerMgr_Status_t status = {0};

  TftUi_GetSettings(&settings);
  if (g_calibration_phase == CONTROL_CALIBRATION_CAPACITOR_BLEED)
  {
    Control_StopPulseOutput();
    for (i = 0U; i < 4U; i++)
    {
      if ((i < count) &&
          (g_cell_voltage[i] >= settings.cell_voltage * CALIBRATION_VSTART_RATIO))
      {
        HAL_GPIO_WritePin(g_balance_ports[i], g_balance_pins[i], GPIO_PIN_SET);
        cell_above_start = 1U;
      }
      else
      {
        HAL_GPIO_WritePin(g_balance_ports[i], g_balance_pins[i], GPIO_PIN_RESET);
      }
    }

    if (cell_above_start != 0U)
    {
      if ((uint32_t)(now - g_calibration_phase_tick_ms) >= 1000U)
      {
        char status_text[64];
        uint32_t threshold_mv =
            Control_RoundVoltageMillivolts(settings.cell_voltage *
                                           CALIBRATION_VSTART_RATIO);
        g_calibration_phase_tick_ms = now;
        snprintf(status_text, sizeof(status_text),
                 "均衡放电：单体低于%lu.%03luV",
                 (unsigned long)(threshold_mv / 1000U),
                 (unsigned long)(threshold_mv % 1000U));
        TftUi_SetCalibrationStatus(status_text);
      }
      return;
    }

    Control_DisableBalanceOutputs();
    g_capacity_start_threshold_v =
      Control_GetRatedTotalVoltage() * CALIBRATION_VSTART_RATIO;
    g_capacity_end_threshold_v =
      Control_GetRatedTotalVoltage() * CALIBRATION_VEND_RATIO;
    if (PowerMgr_StartCalibrationCharge(g_capacity_end_threshold_v,
                                        CALIBRATION_CURRENT_A) == 0U)
    {
      Control_CalibrationFinish("充电器或INA226不可用");
      return;
    }
    g_calibration_phase = CONTROL_CALIBRATION_CAPACITOR_CHARGE;
    g_capacity_integral_started = 0U;
    g_capacity_charge_c = 0.0f;
    {
      char status_text[64];
      uint32_t threshold_mv =
          Control_RoundVoltageMillivolts(g_capacity_start_threshold_v);
      snprintf(status_text, sizeof(status_text), "恒流充电，起点%lu.%03luV",
               (unsigned long)(threshold_mv / 1000U),
               (unsigned long)(threshold_mv % 1000U));
      TftUi_SetCalibrationStatus(status_text);
    }
    return;
  }

  if (g_calibration_phase != CONTROL_CALIBRATION_CAPACITOR_CHARGE)
  {
    return;
  }

  PowerMgr_GetStatus(&status);
  if ((status.ina226_connected == 0U) ||
      (status.measurement_valid == 0U) ||
      ((uint32_t)(now - status.measurement_tick_ms) > 500U))
  {
    Control_CalibrationFinish("INA226测量失效，校准中止");
    return;
  }

  if (g_capacity_integral_started == 0U)
  {
    if (total_voltage >= g_capacity_start_threshold_v)
    {
      g_capacity_integral_started = 1U;
      g_capacity_charge_c = 0.0f;
      g_capacity_start_voltage_v = total_voltage;
      g_last_capacity_integral_ms = now;
      {
        char status_text[64];
        uint32_t threshold_mv =
            Control_RoundVoltageMillivolts(g_capacity_start_threshold_v);
        snprintf(status_text, sizeof(status_text),
                 "已到%lu.%03luV，开始积分",
                 (unsigned long)(threshold_mv / 1000U),
                 (unsigned long)(threshold_mv % 1000U));
        TftUi_SetCalibrationStatus(status_text);
      }
    }
    return;
  }

  if ((status.input_current_a <= 0.0f) ||
      ((uint32_t)(now - status.measurement_tick_ms) > 150U))
  {
    Control_CalibrationFinish("充电电流无效，校准中止");
    return;
  }

  {
    uint32_t elapsed_ms = now - g_last_capacity_integral_ms;
    if (elapsed_ms > (CALIBRATION_SAMPLE_MS * 2U))
    {
      Control_CalibrationFinish("积分采样间隔异常，校准中止");
      return;
    }
    g_capacity_charge_c += status.input_current_a * ((float)elapsed_ms / 1000.0f);
    g_last_capacity_integral_ms = now;
  }

  if (total_voltage >= g_capacity_end_threshold_v)
  {
    float voltage_delta = total_voltage - g_capacity_start_voltage_v;
    float measured_capacitance_f = 0.0f;
    if (!(voltage_delta > 0.0f))
    {
      Control_CalibrationFinish("容量计算异常，校准失败");
      return;
    }
    measured_capacitance_f = g_capacity_charge_c / voltage_delta;
    if (!(measured_capacitance_f > 0.0f) || !(measured_capacitance_f < 1000000.0f))
    {
      Control_CalibrationFinish("容量计算异常，校准失败");
      return;
    }

    TftUi_LoadCalibrationData(&g_calibration_data);
    g_calibration_data.capacitance_f = measured_capacitance_f;
    if (TftUi_SaveCalibrationData(&g_calibration_data) == 0U)
    {
      Control_CalibrationFinish("Flash写入失败");
      return;
    }
    Control_CalibrationFinish("容量校准完成，已保存");
    return;
  }

  if ((uint32_t)(now - g_calibration_phase_tick_ms) >= 1000U)
  {
    uint32_t current_mv = Control_RoundVoltageMillivolts(total_voltage);
    uint32_t target_mv =
        Control_RoundVoltageMillivolts(g_capacity_end_threshold_v);
    g_calibration_phase_tick_ms = now;
    {
      char status_text[64];
      snprintf(status_text, sizeof(status_text), "积分：%lu.%03lu/%lu.%03luV",
               (unsigned long)(current_mv / 1000U),
               (unsigned long)(current_mv % 1000U),
               (unsigned long)(target_mv / 1000U),
               (unsigned long)(target_mv % 1000U));
      TftUi_SetCalibrationStatus(status_text);
    }
  }
}

/* 采集多次测试脉冲压降并保存内阻校准基准。 */
static void Control_VoltageDropCalibrationTask(uint32_t now)
{
  float rated_voltage = Control_GetRatedTotalVoltage();
  float current_voltage = Control_ReadTotalVoltage();

  /* 多次接触测试后取中间三次平均，降低单次接触波动对基准的影响。 */
  if (g_calibration_drop_count >= CALIBRATION_DROP_PULSE_COUNT)
  {
    float sorted[CALIBRATION_DROP_PULSE_COUNT] = {0.0f};
    float reference_drop = 0.0f;
    uint8_t i = 0U;
    uint8_t j = 0U;

    memcpy(sorted, g_calibration_drop_samples, sizeof(sorted));
    for (i = 0U; i < CALIBRATION_DROP_PULSE_COUNT; i++)
    {
      for (j = (uint8_t)(i + 1U); j < CALIBRATION_DROP_PULSE_COUNT; j++)
      {
        if (sorted[j] < sorted[i])
        {
          float swap = sorted[i];
          sorted[i] = sorted[j];
          sorted[j] = swap;
        }
      }
    }
    reference_drop = (sorted[1U] + sorted[2U] + sorted[3U]) / 3.0f;
    if ((reference_drop < CALIBRATION_DROP_MIN_V) ||
        (reference_drop > CALIBRATION_DROP_MAX_V))
    {
      Control_CalibrationFinish("压降异常，请压紧点焊笔");
      return;
    }

    TftUi_LoadCalibrationData(&g_calibration_data);
    g_calibration_data.voltage_drop_v = reference_drop;
    if (TftUi_SaveCalibrationData(&g_calibration_data) == 0U)
    {
      Control_CalibrationFinish("Flash写入失败");
      return;
    }
    Control_CalibrationFinish("内阻基准校准完成");
    return;
  }

  if ((uint32_t)(now - g_calibration_phase_tick_ms) < CALIBRATION_DROP_PULSE_MS)
  {
    return;
  }
  g_calibration_phase_tick_ms = now;
  if ((current_voltage < rated_voltage * CALIBRATION_VDROP_MIN_RATIO) ||
      (current_voltage > rated_voltage * CALIBRATION_VDROP_MAX_RATIO))
  {
    char status_text[64];
    uint32_t target_mv =
        Control_RoundVoltageMillivolts(rated_voltage *
                                       CALIBRATION_VDROP_REFERENCE_RATIO);
    snprintf(status_text, sizeof(status_text), "请充至%lu.%03luV基准",
             (unsigned long)(target_mv / 1000U),
             (unsigned long)(target_mv % 1000U));
    Control_CalibrationFinish(status_text);
    return;
  }

  {
    float voltage_before = current_voltage;
    float voltage_after = 0.0f;
    if (Control_RunTestPulse() == 0U)
    {
      Control_CalibrationFinish("测试脉冲输出失败");
      return;
    }
    voltage_after = Control_ReadTotalVoltage();
    g_calibration_drop_samples[g_calibration_drop_count] = voltage_before - voltage_after;
    if (g_calibration_drop_samples[g_calibration_drop_count] < CALIBRATION_DROP_MIN_V)
    {
      TftUi_NotifyMessage("请压紧点焊笔！", 2000U);
      Control_CalibrationFinish("压降无效，请检查点焊笔和测试片");
      return;
    }
    g_calibration_drop_count++;
    {
      char status_text[40];
      snprintf(status_text, sizeof(status_text), "标准脉冲：%u/%u",
               (unsigned int)g_calibration_drop_count,
               (unsigned int)CALIBRATION_DROP_PULSE_COUNT);
      TftUi_SetCalibrationStatus(status_text);
    }
  }
}

/*
 * Control_StartPwmBurst
 * ------------------------------------------------------------
 * 启动一次点焊 PWM 脉冲，并把当前状态机转入对应的阶段。
 *
 * 这里属于非阻塞控制的关键点：不在循环中直接 delay，而是记住
 * 阶段开始时刻，再由 Control_WeldTask() 在后续扫描周期中按时间
 * 推进。这样既能保持主循环响应，也能避免在超声波/电容放电等
 * 关键波形之间发生阻塞，确保用户触摸与安全保护都能被及时处理。
 */
static void Control_StartPwmBurst(uint16_t duration_ms, ControlWeldPhase_t phase, uint32_t now)
{
  if (duration_ms == 0U)
  {
    g_weld_phase = phase;
    g_weld_phase_tick_ms = now;
    return;
  }
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, WELD_PWM_COMPARE);
  htim1.Instance->CR1 &= ~TIM_CR1_OPM;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Control_StopPulseOutput();
    g_weld_phase = CONTROL_WELD_IDLE;
    PowerMgr_SetWeldInhibit(0U);
    Control_NormalOutputTask();
    TftUi_NotifyMessage("点焊输出故障！", 2000U);
    return;
  }
  g_weld_phase = phase;
  g_weld_phase_tick_ms = now;
}

/*
 * Control_CompensatePulse
 * ------------------------------------------------------------
 * 根据当前电容组电压和校准参数，对点焊脉冲宽度做实时补偿。
 *
 * 由于电池/电容电压变动会直接影响等效供电能力，所以不能简单使用
 * 设置值。这里将理论脉冲时长乘以电压补偿系数，并把输出限制在
 * 0~99 ms 内，避免在低压或高压状态下出现过大脉冲导致焊接异常。
 */
static uint16_t Control_CompensatePulse(uint16_t pulse_ms)
{
  uint32_t adjusted = (uint32_t)((float)pulse_ms * g_weld_compensation + 0.5f);
  if (adjusted > 99U)
  {
    adjusted = 99U;
  }
  return (uint16_t)adjusted;
}

/*
 * Control_FinishWeld
 * ------------------------------------------------------------
 * 结束一轮点焊，将输出关闭并更新焊接统计数据。
 *
 * 这一步不仅清理 PWM 和禁止位，还会在焊接完成后读取最终总电压，
 * 估算本次焊接释放的电容能量，并把计数器写回 TFT UI。这样界面
 * 可以展示本次会话累计次数以及本次焊接的能量值，便于用户观察
 * 设备状态和焊接质量是否正常。
 */
static void Control_FinishWeld(uint32_t now)
{
  Control_StopPulseOutput();
  float voltage_after = Control_ReadTotalVoltage();
  g_weld_phase = CONTROL_WELD_IDLE;
  g_weld_cooldown_until_ms = now + WELD_COOLDOWN_MS;

  /*
   * 焊接结束后恢复充电和均衡权限，说明本轮焊接中的高优先级互锁已经
   * 解除。恢复动作必须发生在 PWM 停止和状态机重置之后，避免在输出
   * 仍在切换期间又再次触发充电与焊接并发。
   */
  PowerMgr_SetWeldInhibit(0U);
  Control_NormalOutputTask();

  if (g_weld_sequence_counted == 0U)
  {
    float energy_j = -1.0f;
    if (g_calibration_data.capacitance_f > 0.0f)
    {
      energy_j = 0.0f;
      if (voltage_after < g_weld_start_voltage_v)
      {
        energy_j = 0.5f * g_calibration_data.capacitance_f *
                   (g_weld_start_voltage_v * g_weld_start_voltage_v -
                    voltage_after * voltage_after);
      }
    }
    TftUi_SetWeldEnergy(energy_j);
    g_session_weld_count++;
    g_total_weld_count++;
    TftUi_SetCounters(g_total_weld_count, g_session_weld_count);
    g_weld_sequence_counted = 1U;
  }
}

/*
 * Control_WeldTask
 * ------------------------------------------------------------
 * 点焊状态机主循环：按时间推进 PULSE_1 -> INTERVAL -> PULSE_2。
 *
 * 每个阶段都保留起始时间戳，系统只需在 Control_Task() 中轮询
 * 当前时间与 g_weld_phase_tick_ms 的差值即可决定是否切换状态。
 * 这样可以在不使用阻塞延时的情况下实现复杂的焊接节拍，避免丢失
 * UI 输入、保护检测和其他背景任务调度。
 */
static void Control_WeldTask(uint32_t now)
{
  uint32_t pulse_duration = 0U;

  /* 点焊各脉冲阶段按时间推进，避免在主循环中用延时等待。 */
  switch (g_weld_phase)
  {
    case CONTROL_WELD_PULSE_1:
      pulse_duration = Control_CompensatePulse(g_weld_pulse1_ms);
      if ((pulse_duration == 0U) ||
          ((uint32_t)(now - g_weld_phase_tick_ms) >= pulse_duration))
      {
        Control_StopPulseOutput();
        if (g_weld_pulse2_ms != 0U)
        {
          g_weld_phase = CONTROL_WELD_INTERVAL;
          g_weld_phase_tick_ms = now;
        }
        else
        {
          Control_FinishWeld(now);
        }
      }
      break;
    case CONTROL_WELD_INTERVAL:
      if ((uint32_t)(now - g_weld_phase_tick_ms) >= g_weld_interval_ms)
      {
        Control_StartPwmBurst(Control_CompensatePulse(g_weld_pulse2_ms),
                              CONTROL_WELD_PULSE_2, now);
      }
      break;
    case CONTROL_WELD_PULSE_2:
      pulse_duration = Control_CompensatePulse(g_weld_pulse2_ms);
      if ((pulse_duration == 0U) ||
          ((uint32_t)(now - g_weld_phase_tick_ms) >= pulse_duration))
      {
        Control_FinishWeld(now);
      }
      break;
    default:
      break;
  }
}

/*
 * Control_BeginWeld
 * ------------------------------------------------------------
 * 在确认触摸事件和电池条件满足要求后，启动一次新的焊接流程。
 *
 * 该函数先判定当前总电压是否达到最低充电阈值，再读取用户设置中
 * 的脉冲参数和补偿校准值，确保输出设置有效。随后通过试脉冲检测
 * 焊笔是否压紧，若检测不到有效压降则拒绝执行，避免误触发和空焊。
 * 最终会设置焊接状态、开启 PWM 并进入非阻塞状态机。
 */
static void Control_BeginWeld(uint32_t now, float total_voltage)
{
  TftUiSettings_t settings = {0};
  float rated_voltage = Control_GetRatedTotalVoltage();

  /* 开始输出前检查电压，并在已校准时用测试脉冲确认焊笔接触。 */
  if (total_voltage < rated_voltage * WELD_VOLTAGE_MIN_RATIO)
  {
    TftUi_NotifyMessage("请充电！", 2000U);
    g_touch_pending = 0U;
    return;
  }

  g_weld_start_voltage_v = total_voltage;
  TftUi_GetSettings(&settings);
  g_weld_pulse1_ms = settings.pulse1_ms;
  g_weld_pulse2_ms = settings.pulse2_ms;
  g_weld_interval_ms = settings.interval_ms;
  if ((g_weld_pulse1_ms == 0U) && (g_weld_pulse2_ms == 0U))
  {
    g_touch_pending = 0U;
    return;
  }

  /*
   * 点焊控制拥有最高优先级：一旦开始焊接，必须立即关闭充电输出和均衡
   * 负载，防止充电电流与焊接脉冲同时流入同一路器件造成电压跌落和
   * 状态竞争。
   *
   * 这里的互锁不是单纯“建议”，而是强制性安全策略：焊接期间 CH_EN
   * 会被 PowerMgr_SetWeldInhibit() 拦掉，直到焊接结束后再由控制逻辑手动
   * 复位并恢复正常输出状态。
   */
  PowerMgr_SetWeldInhibit(1U);
  Control_DisableBalanceOutputs();
  if (g_calibration_data.voltage_drop_v > 0.0f)
  {
    float voltage_before = total_voltage;
    float voltage_after = 0.0f;
    float measured_drop = 0.0f;
    if (Control_RunTestPulse() == 0U)
    {
      g_touch_pending = 0U;
      PowerMgr_SetWeldInhibit(0U);
      Control_NormalOutputTask();
      TftUi_NotifyMessage("点焊输出故障！", 2000U);
      return;
    }
    voltage_after = Control_ReadTotalVoltage();
    measured_drop = voltage_before - voltage_after;
    if ((measured_drop < g_calibration_data.voltage_drop_v * 0.50f) ||
        (measured_drop > g_calibration_data.voltage_drop_v * 1.50f))
    {
      g_touch_pending = 0U;
      PowerMgr_SetWeldInhibit(0U);
      Control_NormalOutputTask();
      TftUi_NotifyMessage("请压紧点焊笔！", 2000U);
      return;
    }
  }

  {
    float voltage_ratio = (rated_voltage * WELD_REFERENCE_VOLTAGE_RATIO) / total_voltage;
    float compensation = voltage_ratio * voltage_ratio;
    if (compensation < WELD_MIN_COMPENSATION)
    {
      compensation = WELD_MIN_COMPENSATION;
    }
    if (compensation > WELD_MAX_COMPENSATION)
    {
      compensation = WELD_MAX_COMPENSATION;
    }
    g_weld_compensation = compensation;
  }

  g_touch_pending = 0U;
  g_weld_sequence_counted = 0U;
  Control_SetWorkState(TFT_UI_STATE_WELDING);
  if (g_weld_pulse1_ms != 0U)
  {
    Control_StartPwmBurst(Control_CompensatePulse(g_weld_pulse1_ms),
                          CONTROL_WELD_PULSE_1, now);
  }
  else
  {
    g_weld_phase = CONTROL_WELD_INTERVAL;
    g_weld_phase_tick_ms = now;
  }
}

/*
 * Control_NormalOutputTask
 * ------------------------------------------------------------
 * 在非校准、非焊接状态下维持正常的充电/均衡控制。
 *
 * 这里会评估各单体电压是否存在压差过大或过高阈值，并决定是否
 * 打开对应均衡 MOSFET；同时根据当前充电状态切换 UI 到 READY、
 * CHARGING 或 BALANCING。它是整个控制器在稳定状态下的“默认动作”，
 * 也是安全状态和焊接状态退出后恢复正常输出时调用的位置。
 */
static void Control_NormalOutputTask(void)
{
  TftUiSettings_t settings = {0};
  PowerMgr_Status_t power_status = {0};
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();
  uint8_t balance_delta_exceeded = 0U;
  uint8_t balance_overvoltage_exceeded = 0U;
  float min_voltage = 999.0f;
  float max_voltage = 0.0f;

  TftUi_GetSettings(&settings);
  PowerMgr_GetStatus(&power_status);
  Control_StopPulseOutput();
  if (Control_CurrentSafetyFaultActive() != 0U)
  {
    Control_DisableBalanceOutputs();
    Control_SetWorkState(TFT_UI_STATE_READY);
    return;
  }

  for (i = 0U; i < count; i++)
  {
    if (g_cell_voltage[i] < min_voltage)
    {
      min_voltage = g_cell_voltage[i];
    }
    if (g_cell_voltage[i] > max_voltage)
    {
      max_voltage = g_cell_voltage[i];
    }
  }

  balance_delta_exceeded = (uint8_t)((count > 1U) &&
                                     (settings.balance_delta_v > 0.0f) &&
                                     ((max_voltage - min_voltage) > settings.balance_delta_v));
  for (i = 0U; i < count; i++)
  {
    if (g_cell_voltage[i] > settings.cell_voltage)
    {
      balance_overvoltage_exceeded = 1U;
      break;
    }
  }

  if ((balance_delta_exceeded != 0U) || (balance_overvoltage_exceeded != 0U))
  {
    float balance_floor = min_voltage + settings.balance_delta_v;
    for (i = 0U; i < 4U; i++)
    {
      uint8_t balance_cell = (uint8_t)((i < count) &&
                                       ((balance_delta_exceeded != 0U &&
                                         (g_cell_voltage[i] > balance_floor)) ||
                                        (g_cell_voltage[i] > settings.cell_voltage)));
      GPIO_PinState pin = ((i < count) && (balance_cell != 0U)) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET;
      HAL_GPIO_WritePin(g_balance_ports[i], g_balance_pins[i], pin);
    }
    Control_SetWorkState(TFT_UI_STATE_BALANCING);
  }
  else
  {
    Control_DisableBalanceOutputs();
    if ((power_status.charge_state == POWER_MGR_CHARGE_CC) ||
        (power_status.charge_state == POWER_MGR_CHARGE_CV) ||
        (power_status.charge_state == POWER_MGR_CHARGE_NEGOTIATING))
    {
      Control_SetWorkState(TFT_UI_STATE_CHARGING);
    }
    else
    {
      Control_SetWorkState(TFT_UI_STATE_READY);
    }
  }
}

/* 更新单体电压和电流，并调度安全、校准及点焊逻辑。 */
static void Control_UpdateMeasurements(uint32_t now)
{
  ControlAdcSampleResult_t sample_result = CONTROL_ADC_SAMPLE_PENDING;
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();
  TftUiSettings_t settings = {0};
  PowerMgr_Status_t power_status = {0};
  float total_voltage = 0.0f;

  TftUi_GetSettings(&settings);
  if ((g_adc_dma_started == 0U) || (count == 0U))
  {
    g_cell_measurements_valid = 0U;
    Control_DisableBalanceOutputs();
    Control_StopPulseOutput();
    return;
  }

  sample_result = Control_PrepareVoltageSampling();
  if (sample_result == CONTROL_ADC_SAMPLE_PENDING)
  {
    return;
  }
  if (sample_result == CONTROL_ADC_SAMPLE_FAILED)
  {
    g_cell_measurements_valid = 0U;
    Control_DisableBalanceOutputs();
    Control_StopPulseOutput();
    return;
  }
  {
    uint16_t samples[4U] = {0U};
    float abs_voltage[4U] = {0.0f};
    float dynamic_vref_mv = Control_GetActualVrefMv();
    Control_GetAdcAverageSnapshot(samples);

    /* 1. 计算各采样点对地的绝对电压（利用动态实时VREF修正） */
    for (i = 0U; i < 4U; i++)
    {
      float adc_pin_mv = ((float)samples[i] * dynamic_vref_mv) / ADC_MAX_COUNTS;
      abs_voltage[i] = (adc_pin_mv * ADC_CELL_VOLTAGE_GAIN) / 1000.0f;
    }

    /* 2. 硬件已经通过LM324完成了差分转换，每个引脚直接对应单体电压 */
    g_cell_voltage[0] = abs_voltage[0];
    g_cell_voltage[1] = abs_voltage[1];
    g_cell_voltage[2] = abs_voltage[2];
    g_cell_voltage[3] = abs_voltage[3];

    /* 3. 计算总电压（各单体电压相加） */
    for (i = 0U; i < count; i++)
    {
      total_voltage += g_cell_voltage[i];
    }
    g_total_voltage_v = total_voltage;
    g_cell_measurements_valid = 1U;

    PowerMgr_GetStatus(&power_status);
    if (power_status.measurement_valid != 0U)
    {
      g_current_a = power_status.input_current_a;
    }
    else if (INA226_IsConnected() != 0U)
    {
      g_current_a = INA226_ReadCurrentA();
    }
    else
    {
      g_current_a = 0.0f;
    }
  }

  /* 每轮测量后先处理安全故障，再运行校准、焊接触发和均衡逻辑。 */
  if (Control_SafetyFaultActive() != 0U)
  {
    g_safety_recovery_sample_count = 0U;
    if (g_fault_latched == 0U)
    {
      TftUi_SetSafetyAlert(TFT_UI_SAFETY_ALERT_CONTROL,
                           1U,
                           "安全保护触发：检查单体电压/电流");
    }
    g_fault_latched = 1U;
    g_touch_pending = 0U;
    if (g_calibration_phase != CONTROL_CALIBRATION_NONE)
    {
      Control_CalibrationFinish("安全保护触发，校准已中止");
    }
    if (g_weld_phase != CONTROL_WELD_IDLE)
    {
      Control_StopPulseOutput();
      g_weld_phase = CONTROL_WELD_IDLE;
      PowerMgr_SetWeldInhibit(0U);
      TftUi_NotifyMessage("安全保护，点焊中止！", 2000U);
    }
    Control_NormalOutputTask();
    return;
  }
  if (g_fault_latched != 0U)
  {
    if (g_safety_recovery_sample_count < CONTROL_SAFETY_RECOVERY_SAMPLE_COUNT)
    {
      g_safety_recovery_sample_count++;
    }
    if (g_safety_recovery_sample_count < CONTROL_SAFETY_RECOVERY_SAMPLE_COUNT)
    {
      Control_NormalOutputTask();
      return;
    }
    g_fault_latched = 0U;
    g_safety_recovery_sample_count = 0U;
    TftUi_SetSafetyAlert(TFT_UI_SAFETY_ALERT_CONTROL, 0U, NULL);
  }

  if ((g_calibration_phase == CONTROL_CALIBRATION_CAPACITOR_BLEED) ||
      (g_calibration_phase == CONTROL_CALIBRATION_CAPACITOR_CHARGE))
  {
    Control_CapacityCalibrationTask(now);
    if (g_calibration_phase != CONTROL_CALIBRATION_NONE)
    {
      Control_SetWorkState((g_calibration_phase == CONTROL_CALIBRATION_CAPACITOR_BLEED) ?
                           TFT_UI_STATE_BALANCING : TFT_UI_STATE_CHARGING);
    }
    return;
  }
  if (g_calibration_phase == CONTROL_CALIBRATION_VDROP_PULSE)
  {
    Control_VoltageDropCalibrationTask(now);
    return;
  }

  if (g_weld_phase != CONTROL_WELD_IDLE)
  {
    return;
  }

  if ((g_touch_pending != 0U) &&
      ((uint32_t)(now - g_touch_trigger_ms) >=
       ((uint32_t)TftUi_GetTriggerTime100ms() * 100U)))
  {
    if ((int32_t)(now - g_weld_cooldown_until_ms) >= 0)
    {
      Control_BeginWeld(now, total_voltage);
    }
  }
  if (g_weld_phase == CONTROL_WELD_IDLE)
  {
    Control_NormalOutputTask();
  }
}

/*
 * Control_StartCalibration
 * ------------------------------------------------------------
 * 根据用户选择启动容量校准或压降校准流程。
 *
 * 进入此函数前，系统必须满足以下条件：
 *   - 当前没有其他正在进行的校准；
 *   - 点焊状态为空闲；
 *   - 电池/电容组电压采样有效；
 *   - 配置参数合法并在安全范围内。
 *
 * 对于容量校准，需要确认外部电源和 INA226 采样有效；
 * 对于压降校准，则要求先准备稳定 ADC 平均采样，再确认电压处在
 * 正常的基准区间。成功后会切换到 g_calibration_phase 状态机，
 * 并把界面切换到相应的显示状态。
 */
uint8_t Control_StartCalibration(TftUiCalibrationType_t type)
{
  TftUiSettings_t settings = {0};
  PowerMgr_Status_t power_status = {0};
  float rated_voltage = 0.0f;
  float voltage = 0.0f;
  uint32_t now = HAL_GetTick();

  if ((type != TFT_UI_CALIBRATION_CAPACITY) &&
      (type != TFT_UI_CALIBRATION_VOLTAGE_DROP))
  {
    return 0U;
  }
  if ((g_calibration_phase != CONTROL_CALIBRATION_NONE) ||
      (g_weld_phase != CONTROL_WELD_IDLE) ||
      (g_cell_measurements_valid == 0U))
  {
    TftUi_SetCalibrationStatus("设备忙或电压采样无效");
    return 0U;
  }
  TftUi_GetSettings(&settings);
  PowerMgr_GetStatus(&power_status);
  if ((settings.capacitor_series == 0U) ||
      (settings.capacitor_series > 4U) ||
      !(settings.cell_voltage >= 1.0f))
  {
    TftUi_SetCalibrationStatus("电容设置无效，无法校准");
    return 0U;
  }

  rated_voltage = settings.cell_voltage * (float)settings.capacitor_series;
  if (type == TFT_UI_CALIBRATION_CAPACITY)
  {
    float calibration_end_voltage = rated_voltage * CALIBRATION_VEND_RATIO;
    if ((calibration_end_voltage < 3.3f) ||
        (calibration_end_voltage > 12.0f))
    {
      TftUi_SetCalibrationStatus("校准目标电压超出3.3-12V范围");
      return 0U;
    }
    if ((power_status.ch224a_connected == 0U) ||
        (power_status.ina226_connected == 0U) ||
        (power_status.measurement_valid == 0U) ||
        ((uint32_t)(now - power_status.measurement_tick_ms) > 500U))
    {
      TftUi_SetCalibrationStatus("请连接电源后重试");
      return 0U;
    }
  }
  else
  {
    /* 开始校准前已要求有有效测量，直接使用最近一次完成的 ADC 平均值。 */
    voltage = g_total_voltage_v;
    if ((voltage < rated_voltage * CALIBRATION_VDROP_MIN_RATIO) ||
        (voltage > rated_voltage * CALIBRATION_VDROP_MAX_RATIO))
    {
      char status_text[64];
      uint32_t target_mv =
          Control_RoundVoltageMillivolts(rated_voltage *
                                         CALIBRATION_VDROP_REFERENCE_RATIO);
      snprintf(status_text, sizeof(status_text), "请充至%lu.%03luV基准",
               (unsigned long)(target_mv / 1000U),
               (unsigned long)(target_mv % 1000U));
      TftUi_SetCalibrationStatus(status_text);
      return 0U;
    }
  }

  g_calibration_previous_work_state = TftUi_GetWorkState();
  PowerMgr_SetCalibrationMode(1U);
  Control_DisableBalanceOutputs();
  Control_StopPulseOutput();
  g_touch_pending = 0U;
  g_calibration_phase_tick_ms = now;
  g_calibration_drop_count = 0U;
  memset(g_calibration_drop_samples, 0, sizeof(g_calibration_drop_samples));
  g_capacity_integral_started = 0U;
  g_capacity_charge_c = 0.0f;

  if (type == TFT_UI_CALIBRATION_CAPACITY)
  {
    g_calibration_phase = CONTROL_CALIBRATION_CAPACITOR_BLEED;
    {
      char status_text[64];
      uint32_t threshold_mv =
          Control_RoundVoltageMillivolts(settings.cell_voltage *
                                         CALIBRATION_VSTART_RATIO);
      snprintf(status_text, sizeof(status_text),
               "均衡放电：单体低于%lu.%03luV",
               (unsigned long)(threshold_mv / 1000U),
               (unsigned long)(threshold_mv % 1000U));
      TftUi_SetCalibrationStatus(status_text);
    }
  }
  else
  {
    g_calibration_phase = CONTROL_CALIBRATION_VDROP_PULSE;
    TftUi_SetCalibrationStatus("焊针压紧标准镍片，准备测试");
  }

  TftUi_SetCalibrationActive(1U);
  return 1U;
}

/* 设置提交后立即要求重新采样并复核所有安全条件。 */
void Control_NotifySettingsChanged(void)
{
  g_settings_recheck_pending = 1U;
  g_safety_recovery_sample_count = 0U;
  g_cell_measurements_valid = 0U;
  g_touch_pending = 0U;
  Control_DisableBalanceOutputs();
  if (g_weld_phase != CONTROL_WELD_IDLE)
  {
    Control_StopPulseOutput();
    g_weld_phase = CONTROL_WELD_IDLE;
    PowerMgr_SetWeldInhibit(0U);
  }
}

/* 请求取消当前校准并安全恢复输出状态。 */
void Control_CancelCalibration(void)
{
  if (g_calibration_phase != CONTROL_CALIBRATION_NONE)
  {
    Control_CalibrationFinish("校准已取消");
  }
}

/* 电源故障时立即关闭充电、均衡、点焊和背光输出。 */
void Control_PowerFailShutdown(void)
{
  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, GPIO_PIN_RESET);
  Control_DisableBalanceOutputs();
  Control_StopPulseOutput();
  g_touch_pending = 0U;
  PowerMgr_StopCalibrationCharge();
  PowerMgr_SetCalibrationMode(0U);
  g_calibration_phase = CONTROL_CALIBRATION_NONE;
  TftUi_SetCalibrationActive(0U);
  g_weld_phase = CONTROL_WELD_IDLE;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);
}

/* 初始化控制状态、ADC DMA 和硬件输出。 */
void Control_Init(void)
{
  uint8_t i = 0U;
  for (i = 0U; i < 4U; i++)
  {
    g_adc_dma_buffer[i] = 0U;
    g_adc_average_sum[i] = 0U;
    g_adc_average[i] = 0U;
  }
  g_last_control_scan_ms = HAL_GetTick();
  g_fault_latched = 0U;
  g_safety_recovery_sample_count = 0U;
  g_touch_pending = 0U;
  g_touch_down = 0U;
  g_session_weld_count = 0U;
  g_total_weld_count = TftUi_LoadPersistentCount();
  g_weld_sequence_counted = 1U;
  g_cell_measurements_valid = 0U;
  g_settings_recheck_pending = 0U;
  g_total_voltage_v = 0.0f;
  g_adc_dma_started = 0U;
  g_adc_average_collecting = 0U;
  g_adc_average_ready = 0U;
  g_adc_average_group_count = 0U;
  g_adc_average_start_ms = 0U;
  g_adc_balance_settling = 0U;
  g_adc_balance_settle_start_ms = 0U;
  g_calibration_phase = CONTROL_CALIBRATION_NONE;
  g_weld_phase = CONTROL_WELD_IDLE;
  TftUi_LoadCalibrationData(&g_calibration_data);
  TftUi_SetCounters(g_total_weld_count, g_session_weld_count);

  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, GPIO_PIN_RESET);
  Control_DisableBalanceOutputs();
  Control_StopPulseOutput();

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(uintptr_t)g_adc_dma_buffer, 4U) == HAL_OK)
  {
    g_adc_dma_started = 1U;
  }
  else
  {
    Error_Handler();
  }
}

/* 在 ADC DMA 完成回调中累计并生成多次采样平均值。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint8_t i = 0U;

  if ((hadc != &hadc1) || (g_adc_average_collecting == 0U))
  {
    return;
  }

  for (i = 0U; i < 4U; i++)
  {
    g_adc_average_sum[i] += g_adc_dma_buffer[i];
  }
  g_adc_average_group_count++;
  if (g_adc_average_group_count >= ADC_AVERAGE_GROUP_COUNT)
  {
    for (i = 0U; i < 4U; i++)
    {
      g_adc_average[i] =
        (uint16_t)((g_adc_average_sum[i] + (ADC_AVERAGE_GROUP_COUNT / 2U)) /
                   ADC_AVERAGE_GROUP_COUNT);
    }
    g_adc_average_collecting = 0U;
    g_adc_average_ready = 1U;
  }
}

/* 判断当前采样、故障和工作阶段是否允许充电。 */
uint8_t Control_IsChargeSafe(void)
{
  return (uint8_t)(g_cell_measurements_valid != 0U &&
                   g_fault_latched == 0U &&
                   Control_SafetyFaultActive() == 0U &&
                   g_calibration_phase == CONTROL_CALIBRATION_NONE &&
                   g_weld_phase == CONTROL_WELD_IDLE);
}

/* 返回当前正在执行均衡的单体位掩码。 */
uint8_t Control_GetBalanceMask(void)
{
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();
  uint8_t mask = 0U;

  for (i = 0U; i < count; i++)
  {
    if (HAL_GPIO_ReadPin(g_balance_ports[i], g_balance_pins[i]) == GPIO_PIN_SET)
    {
      mask |= (uint8_t)(1U << i);
    }
  }
  return mask;
}

/*
 * Control_Task
 * ------------------------------------------------------------
 * 此函数是控制器主循环中的统一入口，每个系统周期都会执行。
 *
 * 它首先检查点焊状态机是否处于活动阶段；若是，则推进焊接节拍，
 * 否则直接进入采样循环。随后根据采样周期或外部设置变更信号触发
 * 一次新的电压/电流更新；这样既能保证实时响应，也能减少无意义的
 * 重复测量和 CPU 占用。
 */
void Control_Task(void)
{
  uint32_t now = HAL_GetTick();
  if (g_weld_phase != CONTROL_WELD_IDLE)
  {
    Control_WeldTask(now);
  }

  if ((g_settings_recheck_pending != 0U) ||
      (g_adc_balance_settling != 0U) ||
      (g_adc_average_collecting != 0U) ||
      (g_adc_average_ready != 0U) ||
      ((uint32_t)(now - g_last_control_scan_ms) >= CONTROL_SAMPLE_PERIOD_MS))
  {
    g_last_control_scan_ms = now;
    g_settings_recheck_pending = 0U;
    Control_UpdateMeasurements(now);
  }
}

/* 获取有效配置范围内指定单体的电压。 */
float Control_GetCellVoltage(uint8_t index)
{
  if ((index >= 4U) || (index >= Control_GetCellCount()))
  {
    return 0.0f;
  }
  return g_cell_voltage[index];
}

/* 获取指定 ADC 单体电压，仅在采样有效时返回数值。 */
float Control_GetAdcCellVoltage(uint8_t index)
{
  if ((index >= 4U) || (g_cell_measurements_valid == 0U))
  {
    return 0.0f;
  }
  return g_cell_voltage[index];
}

/* 获取有效的电容组总电压并通过参数返回。 */
uint8_t Control_GetTotalVoltage(float *total_voltage_v)
{
  if ((total_voltage_v == NULL) || (g_cell_measurements_valid == 0U))
  {
    return 0U;
  }

  *total_voltage_v = g_total_voltage_v;
  return 1U;
}

/* 返回首个超过单体耐压设定值的有效单体测量。 */
uint8_t Control_GetOvervoltageCell(uint8_t *cell_index,
                                   float *cell_voltage_v,
                                   float *limit_voltage_v)
{
  TftUiSettings_t settings = {0};
  uint8_t i = 0U;
  uint8_t count = Control_GetCellCount();

  if ((cell_index == NULL) || (cell_voltage_v == NULL) ||
      (limit_voltage_v == NULL) || (g_cell_measurements_valid == 0U))
  {
    return 0U;
  }

  TftUi_GetSettings(&settings);
  for (i = 0U; i < count; i++)
  {
    if (g_cell_voltage[i] > settings.cell_voltage)
    {
      *cell_index = (uint8_t)(i + 1U);
      *cell_voltage_v = g_cell_voltage[i];
      *limit_voltage_v = settings.cell_voltage;
      return 1U;
    }
  }

  return 0U;
}

/* 返回最近一次更新的电流值，单位为安培。 */
float Control_GetCurrentA(void)
{
  return g_current_a;
}

/*
 * Control_HandleTouch
 * ------------------------------------------------------------
 * 监听触摸开关的电平变化，并在满足条件时记录焊接触发请求。
 *
 * 由于触摸按键本身没有可靠的边沿电路，代码通过 g_touch_down 记录
 * 当前按下状态，只在从“未按下”转为“按下”时建立一次有效触发事件。
 * 这里会检查：焊接状态空闲、校准未进行、故障未锁定，以及冷却时间
 * 已到位，然后设置 g_touch_pending 和 g_touch_trigger_ms。后续在采样
 * 周期中由 Control_UpdateMeasurements() 统一判定是否真正开始焊接。
 */
void Control_HandleTouch(void)
{
  if (HAL_GPIO_ReadPin(Touch_GPIO_Port, Touch_Pin) == GPIO_PIN_RESET)
  {
    if (g_touch_down == 0U)
    {
      g_touch_down = 1U;
      if ((g_weld_phase == CONTROL_WELD_IDLE) &&
          (g_calibration_phase == CONTROL_CALIBRATION_NONE) &&
          (g_fault_latched == 0U) &&
          ((int32_t)(HAL_GetTick() - g_weld_cooldown_until_ms) >= 0))
      {
        g_touch_pending = 1U;
        g_touch_trigger_ms = HAL_GetTick();
      }
    }
  }
  else
  {
    g_touch_down = 0U;
  }
}
