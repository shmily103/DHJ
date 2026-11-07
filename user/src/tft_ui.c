#include "tft_ui.h"
#include "boot_splash.h"
#include "st7789.h"
#include "power_mgr.h"
#include "control.h"
#include "gpio.h"
#include "tim.h"
#include "stm32f1xx_hal_flash.h"
#include <stdio.h>
#include <string.h>

/*
 * tft_ui.c
 * ------------------------------------------------------------
 * 用户界面和 Flash 持久化实现。
 *
 * 该文件是系统与用户交互的核心：
 *   - 管理首页、菜单和校准界面的所有显示区域；
 *   - 响应编码器旋转和点击事件；
 *   - 更新焊接次数、状态和参数；
 *   - 通过 Flash 记录配置和计数，以便断电后恢复。
 *
 * 由于设备是单机嵌入式系统，UI 逻辑与控制逻辑的耦合必须尽量低，
 * 通过 Clean API 和状态同步可以保证显示层只展示状态，而不是直接
 * 执行复杂控制动作。
 */

#define TFT_FLASH_PAGE_61_ADDR 0x0800F400U
#define TFT_FLASH_PAGE_62_ADDR 0x0800F800U
#define TFT_FLASH_PAGE_63_ADDR 0x0800FC00U
#define TFT_FLASH_PAGE_60_ADDR 0x0800F000U
#define TFT_FLASH_MAGIC        0x57454C44U
#define TFT_FLASH_VERSION      0x00000001U
#define TFT_HOME_CARD_MARGIN_X 4U
#define TFT_HOME_CARD_TOP      4U
#define TFT_HOME_CARD_GAP      4U
#define TFT_HOME_CARD_RADIUS   4U
#define TFT_HOME_FOOTER_TOP    168U
#define TFT_HOME_ADC_REFRESH_MS 500U
#define TFT_UI_RGB565(r, g, b) \
  (uint16_t)((((uint16_t)(r) & 0xF8U) << 8) | (((uint16_t)(g) & 0xFCU) << 3) | ((uint16_t)(b) >> 3))
#define TFT_UI_COLOR_BACKGROUND TFT_UI_RGB565(13U, 22U, 38U)
#define TFT_UI_COLOR_CARD_1     TFT_UI_RGB565(255U, 183U, 77U)
#define TFT_UI_COLOR_CARD_2     TFT_UI_RGB565(79U, 195U, 247U)
#define TFT_UI_COLOR_CARD_3     TFT_UI_RGB565(77U, 182U, 172U)
#define TFT_UI_COLOR_CARD_4     TFT_UI_RGB565(149U, 117U, 205U)
#define TFT_UI_COLOR_CARD_5     TFT_UI_RGB565(129U, 199U, 132U)
#define TFT_UI_COLOR_CARD_6     TFT_UI_RGB565(240U, 98U, 146U)
#define TFT_UI_COLOR_CARD_LABEL TFT_UI_RGB565(45U, 57U, 75U)
#define TFT_UI_COLOR_TEXT       TFT_UI_RGB565(18U, 29U, 46U)
#define TFT_UI_COLOR_BALANCE_1  TFT_UI_RGB565(229U, 57U, 53U)
#define TFT_UI_COLOR_BALANCE_2  TFT_UI_RGB565(30U, 136U, 229U)
#define TFT_UI_COLOR_BALANCE_3  TFT_UI_RGB565(67U, 160U, 71U)
#define TFT_UI_COLOR_BALANCE_4  TFT_UI_RGB565(251U, 140U, 0U)
#define TFT_UI_COLOR_CHARGING   TFT_UI_RGB565(0U, 160U, 120U)
#define TFT_UI_COLOR_ROW        TFT_UI_RGB565(29U, 43U, 63U)
#define TFT_UI_COLOR_MENU_TEXT  TFT_UI_RGB565(241U, 246U, 250U)
#define TFT_UI_COLOR_SELECTED   TFT_UI_RGB565(0U, 188U, 212U)
#define TFT_UI_COLOR_EDIT       TFT_UI_RGB565(0U, 188U, 212U)
#define TFT_UI_COLOR_MESSAGE_BACKGROUND TFT_UI_COLOR_CARD_5
#define TFT_UI_COLOR_MESSAGE_TEXT       TFT_UI_COLOR_TEXT
#define TFT_UI_COLOR_ALERT_BACKGROUND  TFT_UI_RGB565(183U, 28U, 28U)
#define TFT_UI_COLOR_ALERT_TEXT         TFT_UI_COLOR_MENU_TEXT

/* 设置和校准数据按页擦除、按字写入；成功写入后再逐字回读校验。 */
/*
 * TftUi_WriteFlashPage
 * ------------------------------------------------------------
 * 把一页 Flash 数据按字写入指定地址。
 *
 * 流程如下：
 *   1. 先解锁 Flash 控制器；
 *   2. 按页擦除目标地址；
 *   3. 逐字写入 data_words[]；
 *   4. 完成后锁定 Flash，避免后续随机写入破坏系统。
 *
 * 它本身不关心具体数据含义，只负责物理层写入；数据结构和内容
 * 由 TftUi_StoreSettingsRecord() 等更高层函数组织，保证 Flash 记录
 * 的格式清晰可验证。
 */
static uint8_t TftUi_WriteFlashPage(uint32_t page_addr, const uint32_t *data_words, uint32_t word_count)
{
  FLASH_EraseInitTypeDef erase_init = {0};
  uint32_t page_error = 0U;
  uint32_t i = 0U;
  HAL_StatusTypeDef flash_status = HAL_OK;

  if (data_words == NULL)
  {
    return 0U;
  }

  flash_status = HAL_FLASH_Unlock();
  if (flash_status != HAL_OK)
  {
    return 0U;
  }

  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = page_addr;
  erase_init.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  for (i = 0U; i < word_count; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, page_addr + (i * 4U), data_words[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return 0U;
    }
  }

  HAL_FLASH_Lock();
  return 1U;
}

/* 从指定地址读取一个 Flash 字。 */
static uint32_t TftUi_ReadFlashWord(uint32_t addr)
{
  return *(volatile uint32_t *)addr;
}

/*
 * TftUi_StoreSettingsRecord
 * ------------------------------------------------------------
 * 把当前配置参数编码成一段固定格式，并写入 Flash。
 *
 * 这里的设计重点是：
 *   - 先把结构体拆成 32-bit words；
 *   - 写入前确保参数符合记录格式；
 *   - 写入完成后立即回读，校验是否一致；
 *   - 若校验失败，认为 Flash 写入异常，返回 0。
 *
 * 这样做可减少断电后恢复时的配置损坏风险；特别是用户调整参数
 * 后，系统重新上电时可以从 Flash 恢复到最后一次稳定配置。
 */
static uint8_t TftUi_StoreSettingsRecord(const TftUiSettings_t *settings, uint32_t page_addr)
{
  uint32_t words[12U] = {0U};
  uint32_t i = 0U;

  if (settings == NULL)
  {
    return 0U;
  }

  words[0] = TFT_FLASH_MAGIC;
  words[1] = TFT_FLASH_VERSION;
  words[2] = (uint32_t)settings->pulse1_ms;
  words[3] = (uint32_t)settings->pulse2_ms;
  words[4] = (uint32_t)settings->interval_ms;
  words[5] = (uint32_t)settings->trigger_time_100ms;
  words[6] = (uint32_t)settings->capacitor_series;
  words[7] = 0U;
  memcpy(&words[7], &settings->cell_voltage, sizeof(float));
  words[8] = 0U;
  memcpy(&words[8], &settings->balance_delta_v, sizeof(float));
  words[9] = 0U;
  memcpy(&words[9], &settings->charge_current_a, sizeof(float));
  words[10] = (uint32_t)settings->charge_enable;
  words[11] = (uint32_t)settings->brightness_percent;

  if (TftUi_WriteFlashPage(page_addr, words, 12U) == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < 12U; i++)
  {
    if (TftUi_ReadFlashWord(page_addr + (i * 4U)) != words[i])
    {
      return 0U;
    }
  }

  return 1U;
}

/* 校验记录标识后从指定 Flash 页恢复设置。 */
static void TftUi_LoadSettingsRecord(TftUiSettings_t *settings, uint32_t page_addr)
{
  uint32_t words[12U] = {0U};
  uint32_t i = 0U;
  uint32_t magic = 0U;
  uint32_t version = 0U;

  if (settings == NULL)
  {
    return;
  }

  for (i = 0U; i < 12U; i++)
  {
    words[i] = TftUi_ReadFlashWord(page_addr + (i * 4U));
  }

  magic = words[0];
  version = words[1];
  if (magic != TFT_FLASH_MAGIC || version != TFT_FLASH_VERSION)
  {
    return;
  }

  settings->pulse1_ms = (uint16_t)words[2];
  settings->pulse2_ms = (uint16_t)words[3];
  settings->interval_ms = (uint16_t)words[4];
  settings->trigger_time_100ms = (uint16_t)words[5];
  settings->capacitor_series = (uint8_t)words[6];
  memcpy(&settings->cell_voltage, &words[7], sizeof(float));
  memcpy(&settings->balance_delta_v, &words[8], sizeof(float));
  memcpy(&settings->charge_current_a, &words[9], sizeof(float));
  settings->charge_enable = (uint8_t)words[10];
  settings->brightness_percent = (uint8_t)words[11];
}

#define TFT_FLASH_PAGE_63_WORDS 256U

/* 次数记录采用追加写入，只有页写满时才擦除，减少 Flash 擦写次数。 */
static uint32_t g_persisted_total_count = 0U;
static uint8_t g_counter_dirty = 0U;
static int32_t g_flash_slot_index = -1;

/*
 * TftUi_LoadPersistentCount
 * ------------------------------------------------------------
 * 从 Flash 的计数页中恢复最后一次保存的焊接总次数。
 *
 * 该函数会：
 *   - 扫描整页数据；
 *   - 找到最后一次非 0xFFFFFFFF 的有效值作为最新计数；
 *   - 记录当前空槽位置（g_flash_slot_index），便于后续追加写入。
 *
 * 这是一种简单但有效的“日志式计数”方案：每次保存时只追加写入，
 * 直到页满才擦除页，减少 Flash 擦除带来的损耗和延迟。
 */
uint32_t TftUi_LoadPersistentCount(void)
{
  uint32_t addr = TFT_FLASH_PAGE_63_ADDR;
  uint32_t latest_count = 0U;
  uint32_t i = 0U;

  g_flash_slot_index = -1;

  for (i = 0U; i < TFT_FLASH_PAGE_63_WORDS; i++)
  {
    uint32_t val = TftUi_ReadFlashWord(addr + (i * 4U));
    if (val != 0xFFFFFFFFU)
    {
      latest_count = val;
    }
    else
    {
      if (g_flash_slot_index < 0)
      {
        g_flash_slot_index = (int32_t)i;
      }
    }
  }

  if (g_flash_slot_index < 0)
  {
    g_flash_slot_index = 0;
  }

  return latest_count;
}

/*
 * TftUi_SavePersistentCount
 * ------------------------------------------------------------
 * 把当前焊接总次数追加写入计数页。
 *
 * 设计逻辑：
 *   - 仅在 g_counter_dirty 为 1 时才执行保存；
 *   - 若当前页已满或索引越界，则先擦除整页；
 *   - 然后将新计数写到下一可用槽位；
 *   - 保存成功后清除脏标记并推进槽位指针。
 *
 * 这样可以延长 Flash 寿命，同时保留最近几次有效计数记录，即使
 * 断电重启也能恢复到最后一次保存值。
 */
void TftUi_SavePersistentCount(uint32_t total_count)
{
  uint32_t addr = TFT_FLASH_PAGE_63_ADDR;

  if (g_counter_dirty == 0U)
  {
    return;
  }

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return;
  }

  if (g_flash_slot_index >= (int32_t)TFT_FLASH_PAGE_63_WORDS || g_flash_slot_index < 0)
  {
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0U;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = TFT_FLASH_PAGE_63_ADDR;
    erase_init.NbPages = 1U;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return;
    }
    g_flash_slot_index = 0;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + ((uint32_t)g_flash_slot_index * 4U), total_count) == HAL_OK)
  {
    g_persisted_total_count = total_count;
    g_counter_dirty = 0U;
    g_flash_slot_index++;
  }

  HAL_FLASH_Lock();
}

/*
 * TftUi_LoadCalibrationData
 * ------------------------------------------------------------
 * 从 Flash 中恢复容量和压降校准数据。
 *
 * 它检查魔术字和版本号是否匹配；只有在有效标记存在时才从
 * 固定地址读取 2 个 float 数据。如果数据不合法或为 0，则视为
 * 没有安装有效校准参数，并将数据清零。
 *
 * 这使得校准结果能在设备重启后继续生效，而不需要每次重新校准。
 */
void TftUi_LoadCalibrationData(TftUiCalibrationData_t *data)
{
  uint32_t magic = 0U;
  uint32_t version = 0U;
  uint32_t words[2U] = {0U};

  if (data == NULL)
  {
    return;
  }

  magic = TftUi_ReadFlashWord(TFT_FLASH_PAGE_60_ADDR);
  version = TftUi_ReadFlashWord(TFT_FLASH_PAGE_60_ADDR + 4U);
  if ((magic != TFT_FLASH_MAGIC) || (version != TFT_FLASH_VERSION))
  {
    return;
  }

  words[0] = TftUi_ReadFlashWord(TFT_FLASH_PAGE_60_ADDR + 8U);
  words[1] = TftUi_ReadFlashWord(TFT_FLASH_PAGE_60_ADDR + 12U);
  memcpy(&data->capacitance_f, &words[0], sizeof(float));
  memcpy(&data->voltage_drop_v, &words[1], sizeof(float));
  if (!(data->capacitance_f > 0.0f))
  {
    data->capacitance_f = 0.0f;
  }
  if (!(data->voltage_drop_v > 0.0f))
  {
    data->voltage_drop_v = 0.0f;
  }
}

/*
 * TftUi_SaveCalibrationData
 * ------------------------------------------------------------
 * 把容量和压降校准结果持久化到 Flash。
 *
 * 该函数会：
 *   - 检查参数合法性；
 *   - 组织成记录结构；
 *   - 调用底层写入函数写入固定页；
 *   - 之后可在再次上电时从 Flash 读取并恢复。
 *
 * 由于校准数据直接影响功率和电压计算，因此它必须被写入完全且
 * 具备有效 magic/version 校验，才能被系统接受。
 */
uint8_t TftUi_SaveCalibrationData(const TftUiCalibrationData_t *data)
{
  uint32_t words[4U] = {0U};

  if (data == NULL)
  {
    return 0U;
  }

  words[0] = TFT_FLASH_MAGIC;
  words[1] = TFT_FLASH_VERSION;
  memcpy(&words[2], &data->capacitance_f, sizeof(float));
  memcpy(&words[3], &data->voltage_drop_v, sizeof(float));
  return TftUi_WriteFlashPage(TFT_FLASH_PAGE_60_ADDR, words, 4U);
}

/* 优先从主记录恢复设置，无效时读取备份记录。 */
static void TftUi_LoadPersistentSettings(TftUiSettings_t *settings)
{
  TftUiSettings_t candidate = {0};
  if (settings == NULL)
  {
    return;
  }

  /* 优先读取主记录；无有效设置时再尝试备份页。 */
  TftUi_LoadSettingsRecord(&candidate, TFT_FLASH_PAGE_61_ADDR);
  if (candidate.pulse1_ms != 0U || candidate.pulse2_ms != 0U || candidate.interval_ms != 0U || candidate.trigger_time_100ms != 0U)
  {
    *settings = candidate;
    return;
  }

  TftUi_LoadSettingsRecord(&candidate, TFT_FLASH_PAGE_62_ADDR);
  if (candidate.pulse1_ms != 0U || candidate.pulse2_ms != 0U || candidate.interval_ms != 0U || candidate.trigger_time_100ms != 0U)
  {
    *settings = candidate;
  }
}

/* 将设置分别写入主记录页和备份记录页。 */
static uint8_t TftUi_SavePersistentSettings(const TftUiSettings_t *settings)
{
  uint8_t primary_saved = 0U;
  uint8_t backup_saved = 0U;

  if (settings == NULL)
  {
    return 0U;
  }

  primary_saved = TftUi_StoreSettingsRecord(settings, TFT_FLASH_PAGE_61_ADDR);
  backup_saved = TftUi_StoreSettingsRecord(settings, TFT_FLASH_PAGE_62_ADDR);
  return (uint8_t)(primary_saved || backup_saved);
}

static TftUiScreen_t g_screen = TFT_UI_SCREEN_HOME;
static TftUiWorkState_t g_work_state = TFT_UI_STATE_READY;
static uint8_t g_menu_index = 0;
static uint8_t g_edit_mode = 0;
static char g_msg[64] = {0};
static uint32_t g_msg_deadline_ms = 0;
static char g_safety_alert_messages[TFT_UI_SAFETY_ALERT_COUNT][64] = {{0}};
static uint8_t g_safety_alert_mask = 0U;
static uint8_t g_calibration_selection = 0U;
static uint8_t g_calibration_active = 0U;
static float g_weld_energy_j = 0.0f;
static uint8_t g_weld_energy_valid = 0U;

static TftUiSettings_t g_settings =
{
  .pulse1_ms = 10,
  .pulse2_ms = 15,
  .interval_ms = 20,
  .trigger_time_100ms = 100,
  .capacitor_series = 4,
  .cell_voltage = 2.7f,
  .balance_delta_v = 0.08f,
  .charge_current_a = 3.0f,
  .charge_enable = 0,
  .brightness_percent = 80U
};

static TftUiSettings_t g_saved_settings =
{
  .pulse1_ms = 10,
  .pulse2_ms = 15,
  .interval_ms = 20,
  .trigger_time_100ms = 15,
  .capacitor_series = 4,
  .cell_voltage = 2.7f,
  .balance_delta_v = 0.08f,
  .charge_current_a = 3.0f,
  .charge_enable = 0,
  .brightness_percent = 80U
};

static const char *g_menu_names[TFT_UI_MENU_ITEMS] =
{
  "脉冲 1", "脉冲 2", "脉冲间隔", "触发时间", "电容串数", "单体耐压",
  "均衡压差", "充电电流", "充电开关", "亮度", "系统校准", "保存", "退出"
};

static uint8_t g_last_home_redraw = 0;
static TftUiSettings_t g_home_drawn_settings = {0};
static uint32_t g_home_drawn_total_count = 0U;
static uint32_t g_home_drawn_session_count = 0U;
static uint32_t g_home_adc_refresh_tick_ms = 0U;
static char g_home_drawn_cell_values[4][8] = {{0}};
static uint8_t g_home_drawn_balance_mask = 0xFFU;
static uint8_t g_home_drawn_charging = 0xFFU;
static char g_home_drawn_total_value[8] = {0};
static char g_home_drawn_power_value[8] = {0};
static uint8_t g_menu_needs_full_redraw = 1U;
static uint8_t g_dirty = 1;
static uint32_t g_total_weld_count = 0U;
static uint32_t g_session_weld_count = 0U;

static void TftUi_DrawMessage(void);

/* 保存尚未写入 Flash 的焊接次数并返回保存结果。 */
uint8_t TftUi_SavePendingCount(void)
{
  if (g_counter_dirty == 0U)
  {
    return 1U;
  }
  TftUi_SavePersistentCount(g_total_weld_count);
  return (uint8_t)(g_counter_dirty == 0U);
}

/* 将整数限制在指定的闭区间内。 */
static int32_t TftUi_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

/* 根据亮度设置更新 TFT 背光 PWM 占空比。 */
static void TftUi_ApplyBrightness(void)
{
  uint32_t duty = 0U;
  uint32_t brightness = (uint32_t)g_settings.brightness_percent;

  if (brightness > 100U)
  {
    brightness = 100U;
  }
  if (brightness < 10U)
  {
    brightness = 10U;
  }

  duty = (brightness * ((uint32_t)htim2.Init.Period + 1U)) / 100U;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

/* 将浮点数限制在指定的闭区间内。 */
static float TftUi_ClampF(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

/* 按菜单当前选项和旋转增量修改设置值。 */
static void TftUi_AdjustSelectedValue(int32_t delta)
{
  switch (g_menu_index)
  {
    case 0:
      g_settings.pulse1_ms = TftUi_ClampI32((int32_t)g_settings.pulse1_ms + delta, 0, 99);
      break;
    case 1:
      g_settings.pulse2_ms = TftUi_ClampI32((int32_t)g_settings.pulse2_ms + delta, 0, 99);
      break;
    case 2:
      g_settings.interval_ms = TftUi_ClampI32((int32_t)g_settings.interval_ms + delta, 0, 99);
      break;
    case 3:
      g_settings.trigger_time_100ms = TftUi_ClampI32((int32_t)g_settings.trigger_time_100ms + delta * 5, 0, 990);
      break;
    case 4:
      g_settings.capacitor_series = TftUi_ClampI32((int32_t)g_settings.capacitor_series + delta, 2, 4);
      break;
    case 5:
      g_settings.cell_voltage = TftUi_ClampF(g_settings.cell_voltage + (float)delta * 0.01f, 1.0f, 3.3f);
      break;
    case 6:
      g_settings.balance_delta_v =
        (float)TftUi_ClampI32((int32_t)(g_settings.balance_delta_v * 1000.0f + 0.5f) + delta,
                              0, 500) / 1000.0f;
      break;
    case 7:
      g_settings.charge_current_a = TftUi_ClampF(g_settings.charge_current_a + (float)delta * 0.1f, 0.0f, 8.0f);
      break;
    case 8:
      g_settings.charge_enable = TftUi_ClampI32((int32_t)g_settings.charge_enable + delta, 0, 1);
      break;
    case 9:
      g_settings.brightness_percent = (uint8_t)TftUi_ClampI32((int32_t)g_settings.brightness_percent + delta * 1, 1, 100);
      break;
    default:
      break;
  }
}

/* 保存当前编辑设置并应用已提交的背光和工作状态。 */
static uint8_t TftUi_SaveSettings(void)
{
  /* 菜单中的编辑值只有确认保存后才成为已提交设置。 */
  if (TftUi_SavePersistentSettings(&g_settings) == 0U)
  {
    TftUi_NotifyMessage("设置保存失败", 2000U);
    return 0U;
  }

  g_saved_settings = g_settings;
  Control_NotifySettingsChanged();
  TftUi_ApplyBrightness();
  TftUi_SetWorkState(TFT_UI_STATE_READY);
  TftUi_NotifyMessage("设置已保存", 1200U);
  return 1U;
}

/* 丢弃未保存的编辑并恢复最近一次已保存设置。 */
static void TftUi_DiscardSettings(void)
{
  g_settings = g_saved_settings;
  TftUi_ApplyBrightness();
}

/* 初始化界面状态并从 Flash 恢复设置和计数。 */
void TftUi_Init(void)
{
  g_screen = TFT_UI_SCREEN_HOME;
  g_work_state = TFT_UI_STATE_READY;
  g_menu_index = 0;
  g_edit_mode = 0;
  g_msg[0] = '\0';
  g_msg_deadline_ms = 0;
  memset(g_safety_alert_messages, 0, sizeof(g_safety_alert_messages));
  g_safety_alert_mask = 0U;
  TftUi_LoadPersistentSettings(&g_saved_settings);
  g_settings = g_saved_settings;
  g_total_weld_count = TftUi_LoadPersistentCount();
  g_persisted_total_count = g_total_weld_count;
  g_counter_dirty = 0U;
  g_session_weld_count = 0U;
  g_menu_needs_full_redraw = 1U;
  TftUi_ApplyBrightness();
  g_dirty = 1;
}

/* 切换界面页面并标记需要重绘的内容。 */
void TftUi_SetScreen(TftUiScreen_t screen)
{
  if (g_screen != screen)
  {
    g_menu_needs_full_redraw = 1U;
  }
  g_screen = screen;
  if (screen == TFT_UI_SCREEN_HOME)
  {
    g_menu_index = 0;
    g_edit_mode = 0;
  }
  g_dirty = 1;
}

/* 更新设备工作状态并请求界面刷新。 */
void TftUi_SetWorkState(TftUiWorkState_t state)
{
  if (g_work_state == state)
  {
    return;
  }

  g_work_state = state;
  g_dirty = 1;
}

/* 返回当前设备工作状态。 */
TftUiWorkState_t TftUi_GetWorkState(void)
{
  return g_work_state;
}

/* 将当前已保存设置复制给调用方。 */
void TftUi_GetSettings(TftUiSettings_t *settings)
{
  if (settings != NULL)
  {
    *settings = g_saved_settings;
  }
}

/* 更新累计及本次会话焊接次数。 */
void TftUi_SetCounters(uint32_t total_count, uint32_t session_count)
{
  g_total_weld_count = total_count;
  g_session_weld_count = session_count;
  g_counter_dirty = (uint8_t)(g_total_weld_count != g_persisted_total_count);
  g_dirty = 1;
}

/* 将累计和会话焊接次数写入非空输出参数。 */
void TftUi_GetCounters(uint32_t *total_count, uint32_t *session_count)
{
  if (total_count != NULL)
  {
    *total_count = g_total_weld_count;
  }
  if (session_count != NULL)
  {
    *session_count = g_session_weld_count;
  }
}

/* 返回以 100 毫秒为单位的触发等待时间。 */
uint16_t TftUi_GetTriggerTime100ms(void)
{
  return g_settings.trigger_time_100ms;
}

/* 显示指定时长的界面提示消息。 */
void TftUi_NotifyMessage(const char *msg, uint16_t duration_ms)
{
  if (msg == NULL)
  {
    return;
  }

  snprintf(g_msg, sizeof(g_msg), "%s", msg);
  g_msg_deadline_ms = HAL_GetTick() + duration_ms;
  g_dirty = 1;
}

/* 显示或清除由对应保护模块持有的持续安全提示。 */
void TftUi_SetSafetyAlert(TftUiSafetyAlertSource_t source, uint8_t active, const char *msg)
{
  uint8_t source_mask = 0U;

  if ((uint32_t)source >= (uint32_t)TFT_UI_SAFETY_ALERT_COUNT)
  {
    return;
  }

  source_mask = (uint8_t)(1U << source);
  if (active != 0U)
  {
    if (msg == NULL)
    {
      return;
    }
    snprintf(g_safety_alert_messages[source], sizeof(g_safety_alert_messages[source]), "%s", msg);
    g_safety_alert_mask |= source_mask;
  }
  else
  {
    g_safety_alert_messages[source][0] = '\0';
    g_safety_alert_mask &= (uint8_t)~source_mask;
  }
  g_dirty = 1U;
}

/* 更新最近一次焊接能量及其有效标志。 */
void TftUi_SetWeldEnergy(float energy_j)
{
  g_weld_energy_valid = (energy_j >= 0.0f) ? 1U : 0U;
  g_weld_energy_j = (g_weld_energy_valid != 0U) ? energy_j : 0.0f;
  g_dirty = 1U;
}

/* 将校准状态作为限时提示显示在界面上。 */
void TftUi_SetCalibrationStatus(const char *status)
{
  if (status != NULL)
  {
    TftUi_NotifyMessage(status, 2000U);
  }
}

/* 更新校准进行状态并请求界面刷新。 */
void TftUi_SetCalibrationActive(uint8_t active)
{
  g_calibration_active = (active != 0U) ? 1U : 0U;
  g_dirty = 1U;
}

/* 切换到首页并重置菜单编辑状态。 */
void TftUi_ShowHome(void)
{
  g_screen = TFT_UI_SCREEN_HOME;
  g_menu_index = 0;
  g_edit_mode = 0;
  g_menu_needs_full_redraw = 1U;
  g_dirty = 1;
}

/* 切换到设置菜单并重置选中项和编辑状态。 */
void TftUi_ShowMenu(void)
{
  g_screen = TFT_UI_SCREEN_MENU;
  g_menu_index = 0;
  g_edit_mode = 0;
  g_menu_needs_full_redraw = 1U;
  g_dirty = 1;
}

/* 根据当前页面和菜单选项处理一次确认键操作。 */
void TftUi_ProcessClick(void)
{
  if (g_screen == TFT_UI_SCREEN_CALIBRATION)
  {
    if (g_calibration_active != 0U)
    {
      Control_CancelCalibration();
      g_calibration_active = 0U;
      TftUi_SetCalibrationStatus("校准已取消");
    }
    else if (g_calibration_selection == 2U)
    {
      g_screen = TFT_UI_SCREEN_MENU;
      g_menu_needs_full_redraw = 1U;
    }
    else
    {
      TftUiCalibrationType_t type = (g_calibration_selection == 0U) ?
                                    TFT_UI_CALIBRATION_CAPACITY :
                                    TFT_UI_CALIBRATION_VOLTAGE_DROP;
      g_calibration_active = Control_StartCalibration(type);
    }
    g_dirty = 1U;
    return;
  }

  if (g_screen == TFT_UI_SCREEN_HOME)
  {
    return;
  }

  if (g_menu_index == 11U)
  {
    if (TftUi_SaveSettings() != 0U)
    {
      TftUi_ShowHome();
    }
    return;
  }

  if (g_menu_index == 12U)
  {
    TftUi_DiscardSettings();
    TftUi_ShowHome();
    return;
  }

  if (g_menu_index == 10U)
  {
    g_screen = TFT_UI_SCREEN_CALIBRATION;
    g_calibration_selection = 0U;
    g_calibration_active = 0U;
    g_dirty = 1U;
    return;
  }

  if (g_edit_mode == 0)
  {
    g_edit_mode = 1;
  }
  else
  {
    g_edit_mode = 0;
  }
  g_dirty = 1;
}

/* 根据页面和编辑状态分发旋钮旋转、短按及长按事件。 */
void TftUi_ProcessEncoderEvent(EncoderEvent_t event)
{
  /* 长按统一用于退出/返回；短按和旋转按页面及编辑状态分流。 */
  if (event == ENCODER_EVENT_LONG_PRESS)
  {
    if (g_screen == TFT_UI_SCREEN_CALIBRATION)
    {
      if (g_calibration_active != 0U)
      {
        Control_CancelCalibration();
        g_calibration_active = 0U;
      }
      g_screen = TFT_UI_SCREEN_MENU;
      g_menu_needs_full_redraw = 1U;
      g_dirty = 1U;
    }
    else if (g_screen == TFT_UI_SCREEN_HOME)
    {
      TftUi_ShowMenu();
    }
    else
    {
      TftUi_DiscardSettings();
      TftUi_ShowHome();
    }
    return;
  }

  if (g_screen == TFT_UI_SCREEN_HOME)
  {
    return;
  }

  if (event == ENCODER_EVENT_CLICK)
  {
    TftUi_ProcessClick();
    return;
  }

  if (g_edit_mode == 1)
  {
    if (event == ENCODER_EVENT_CW)
    {
      TftUi_AdjustSelectedValue(1);
    }
    else if (event == ENCODER_EVENT_CCW)
    {
      TftUi_AdjustSelectedValue(-1);
    }
    else if (event == ENCODER_EVENT_CW_FAST)
    {
      TftUi_AdjustSelectedValue(10);
    }
    else if (event == ENCODER_EVENT_CCW_FAST)
    {
      TftUi_AdjustSelectedValue(-10);
    }
    g_dirty = 1;
    return;
  }

  if (g_screen == TFT_UI_SCREEN_CALIBRATION)
  {
    if ((event == ENCODER_EVENT_CW) || (event == ENCODER_EVENT_CW_FAST))
    {
      g_calibration_selection = (uint8_t)((g_calibration_selection + 1U) % 3U);
    }
    else if ((event == ENCODER_EVENT_CCW) || (event == ENCODER_EVENT_CCW_FAST))
    {
      g_calibration_selection = (g_calibration_selection == 0U) ?
                                2U : (uint8_t)(g_calibration_selection - 1U);
    }
    g_dirty = 1U;
    return;
  }

  switch (event)
  {
    case ENCODER_EVENT_CW:
    case ENCODER_EVENT_CW_FAST:
      g_menu_index = (g_menu_index + 1U) % TFT_UI_MENU_ITEMS;
      break;
    case ENCODER_EVENT_CCW:
    case ENCODER_EVENT_CCW_FAST:
      if (g_menu_index == 0)
      {
        g_menu_index = TFT_UI_MENU_ITEMS - 1;
      }
      else
      {
        g_menu_index--;
      }
      break;
    default:
      break;
  }
  g_dirty = 1;
}

/* 绘制首页卡片布局周围的背景区域。 */
static void TftUi_DrawHomeBackground(void)
{
  const uint16_t box_w = (ST7789_WIDTH - 2U * TFT_HOME_CARD_MARGIN_X -
                          2U * TFT_HOME_CARD_GAP) / 3U;
  const uint16_t box_h = (TFT_HOME_FOOTER_TOP - TFT_HOME_CARD_TOP -
                          TFT_HOME_CARD_GAP) / 2U;
  const uint16_t row_gap_y = (uint16_t)(TFT_HOME_CARD_TOP + box_h);
  const uint16_t third_card_x = (uint16_t)(TFT_HOME_CARD_MARGIN_X +
                                           2U * (box_w + TFT_HOME_CARD_GAP));

  ST7789_FillRect(0U, 0U, ST7789_WIDTH, TFT_HOME_CARD_TOP, TFT_UI_COLOR_BACKGROUND);
  ST7789_FillRect(0U, row_gap_y, ST7789_WIDTH, TFT_HOME_CARD_GAP, TFT_UI_COLOR_BACKGROUND);
  ST7789_FillRect(0U, TFT_HOME_FOOTER_TOP, ST7789_WIDTH,
                  ST7789_HEIGHT - TFT_HOME_FOOTER_TOP, TFT_UI_COLOR_BACKGROUND);

  ST7789_FillRect(0U, TFT_HOME_CARD_TOP, TFT_HOME_CARD_MARGIN_X, box_h * 2U + TFT_HOME_CARD_GAP,
                  TFT_UI_COLOR_BACKGROUND);
  ST7789_FillRect(TFT_HOME_CARD_MARGIN_X + box_w, TFT_HOME_CARD_TOP, TFT_HOME_CARD_GAP,
                  box_h * 2U + TFT_HOME_CARD_GAP, TFT_UI_COLOR_BACKGROUND);
  ST7789_FillRect(TFT_HOME_CARD_MARGIN_X + 2U * box_w + TFT_HOME_CARD_GAP,
                  TFT_HOME_CARD_TOP, TFT_HOME_CARD_GAP, box_h * 2U + TFT_HOME_CARD_GAP,
                  TFT_UI_COLOR_BACKGROUND);
  ST7789_FillRect(third_card_x + box_w, TFT_HOME_CARD_TOP,
                  ST7789_WIDTH - (third_card_x + box_w),
                  box_h * 2U + TFT_HOME_CARD_GAP, TFT_UI_COLOR_BACKGROUND);
}

/* 绘制首页电压、功率、参数和焊接计数信息。 */
void TftUi_DrawHome(void)
{
  /* 首页使用整帧命令队列，避免局部内容通过阻塞式 SPI 绘制。 */
  uint8_t i = 0U;
  uint8_t full_redraw = 1U;
  uint16_t box_x = 0U;
  uint16_t box_y = 0U;
  const uint16_t box_w = (ST7789_WIDTH - 2U * TFT_HOME_CARD_MARGIN_X - 2U * TFT_HOME_CARD_GAP) / 3U;
  const uint16_t box_h = (TFT_HOME_FOOTER_TOP - TFT_HOME_CARD_TOP - TFT_HOME_CARD_GAP) / 2U;
  uint16_t cell1_cv = (uint16_t)(Control_GetAdcCellVoltage(0U) * 100.0f + 0.5f);
  uint16_t cell2_cv = (uint16_t)(Control_GetAdcCellVoltage(1U) * 100.0f + 0.5f);
  uint16_t cell3_cv = (uint16_t)(Control_GetAdcCellVoltage(2U) * 100.0f + 0.5f);
  uint16_t cell4_cv = (uint16_t)(Control_GetAdcCellVoltage(3U) * 100.0f + 0.5f);
  uint16_t total_cv = 0U;
  TftUiSettings_t settings = {0};
  PowerMgr_Status_t power_status = {0};
  char value[32];
  char cell_values[4][8];
  char total_value[8];
  char power_value[8];
  const uint8_t balance_mask = Control_GetBalanceMask();
  uint8_t charging_active = 0U;
  const uint16_t balance_colors[4U] =
  {
    TFT_UI_COLOR_BALANCE_1,
    TFT_UI_COLOR_BALANCE_2,
    TFT_UI_COLOR_BALANCE_3,
    TFT_UI_COLOR_BALANCE_4
  };
  const uint16_t cell_positions_x[4U] = {88U, 208U, 88U, 208U};
  const uint16_t cell_positions_y[4U] =
  {
    TFT_HOME_FOOTER_TOP + 4U,
    TFT_HOME_FOOTER_TOP + 4U,
    TFT_HOME_FOOTER_TOP + 25U,
    TFT_HOME_FOOTER_TOP + 25U
  };
  const uint16_t block_colors[6U] =
  {
    TFT_UI_COLOR_CARD_1,
    TFT_UI_COLOR_CARD_2,
    TFT_UI_COLOR_CARD_3,
    TFT_UI_COLOR_CARD_4,
    TFT_UI_COLOR_CARD_5,
    TFT_UI_COLOR_CARD_6
  };

  TftUi_GetSettings(&settings);
  if (settings.capacitor_series >= 1U)
  {
    total_cv = cell1_cv;
  }
  if (settings.capacitor_series >= 2U)
  {
    total_cv = (uint16_t)(total_cv + cell2_cv);
  }
  if (settings.capacitor_series >= 3U)
  {
    total_cv = (uint16_t)(total_cv + cell3_cv);
  }
  if (settings.capacitor_series >= 4U)
  {
    total_cv = (uint16_t)(total_cv + cell4_cv);
  }

  snprintf(cell_values[0], sizeof(cell_values[0]), "%u.%02uV",
           (unsigned int)(cell1_cv / 100U), (unsigned int)(cell1_cv % 100U));
  snprintf(cell_values[1], sizeof(cell_values[1]), "%u.%02uV",
           (unsigned int)(cell2_cv / 100U), (unsigned int)(cell2_cv % 100U));
  snprintf(cell_values[2], sizeof(cell_values[2]), "%u.%02uV",
           (unsigned int)(cell3_cv / 100U), (unsigned int)(cell3_cv % 100U));
  snprintf(cell_values[3], sizeof(cell_values[3]), "%u.%02uV",
           (unsigned int)(cell4_cv / 100U), (unsigned int)(cell4_cv % 100U));
  snprintf(total_value, sizeof(total_value), "%02u.%02uV",
           (unsigned int)(total_cv / 100U), (unsigned int)(total_cv % 100U));
  PowerMgr_GetStatus(&power_status);
  charging_active = (uint8_t)(
    HAL_GPIO_ReadPin(CH_EN_GPIO_Port, CH_EN_Pin) == GPIO_PIN_SET);
  if ((power_status.ina226_connected != 0U) &&
      (power_status.measurement_valid != 0U))
  {
    uint16_t power_dW = 0U;
    if (power_status.input_power_w >= 999.9f)
    {
      power_dW = 9999U;
    }
    else if (power_status.input_power_w > 0.0f)
    {
      power_dW = (uint16_t)(power_status.input_power_w * 10.0f + 0.5f);
    }
    snprintf(power_value, sizeof(power_value), "%2u.%u",
             (unsigned int)(power_dW / 10U), (unsigned int)(power_dW % 10U));
  }
  else
  {
    (void)strncpy(power_value, " --.-", sizeof(power_value) - 1U);
    power_value[sizeof(power_value) - 1U] = '\0';
  }

  if (full_redraw != 0U)
  {
    ST7789_BeginFrame(TFT_UI_COLOR_BACKGROUND);
    TftUi_DrawHomeBackground();
    ST7789_DrawString(4U, TFT_HOME_FOOTER_TOP + 48U, "电压",
                      TFT_UI_COLOR_CARD_LABEL, TFT_UI_COLOR_BACKGROUND);
    ST7789_DrawString(228U, TFT_HOME_FOOTER_TOP + 48U, "焊功",
                      TFT_UI_COLOR_CARD_LABEL, TFT_UI_COLOR_BACKGROUND);
  }

  for (i = 0U; i < 4U; i++)
  {
    uint16_t cell_label_color = ((balance_mask & (uint8_t)(1U << i)) != 0U) ?
                                balance_colors[i] : TFT_UI_COLOR_CARD_LABEL;
    if ((i < settings.capacitor_series) &&
        ((full_redraw != 0U) || (balance_mask != g_home_drawn_balance_mask)))
    {
      ST7789_DrawString((uint16_t)(cell_positions_x[i] - 36U),
                        cell_positions_y[i],
                        (i == 0U) ? "C1 " : (i == 1U) ? "C2 " :
                        (i == 2U) ? "C3 " : "C4 ",
                        cell_label_color, TFT_UI_COLOR_BACKGROUND);
    }
  }

  for (i = 0U; i < 6U; i++)
  {
    const char *label = "";
    uint8_t redraw_card = full_redraw;
    uint16_t row = (uint16_t)(i / 3U);
    uint16_t col = (uint16_t)(i % 3U);

    box_x = TFT_HOME_CARD_MARGIN_X + col * (box_w + TFT_HOME_CARD_GAP);
    box_y = TFT_HOME_CARD_TOP + row * (box_h + TFT_HOME_CARD_GAP);

    switch (i)
    {
      case 0U:
        label = "脉冲 1";
        snprintf(value, sizeof(value), "%d ms", g_settings.pulse1_ms);
        redraw_card = (uint8_t)(redraw_card || g_home_drawn_settings.pulse1_ms != g_settings.pulse1_ms);
        break;
      case 1U:
        label = "脉冲 2";
        snprintf(value, sizeof(value), "%d ms", g_settings.pulse2_ms);
        redraw_card = (uint8_t)(redraw_card || g_home_drawn_settings.pulse2_ms != g_settings.pulse2_ms);
        break;
      case 2U:
        label = "间隔";
        snprintf(value, sizeof(value), "%d ms", g_settings.interval_ms);
        redraw_card = (uint8_t)(redraw_card || g_home_drawn_settings.interval_ms != g_settings.interval_ms);
        break;
      case 3U:
        label = "等待";
        snprintf(value, sizeof(value), "%u.%u s",
                 (unsigned int)(g_settings.trigger_time_100ms / 10U),
                 (unsigned int)(g_settings.trigger_time_100ms % 10U));
        redraw_card = (uint8_t)(redraw_card ||
                                g_home_drawn_settings.trigger_time_100ms != g_settings.trigger_time_100ms);
        break;
      case 4U:
        label = "总次数";
        snprintf(value, sizeof(value), "%lu", (unsigned long)g_total_weld_count);
        redraw_card = (uint8_t)(redraw_card || g_home_drawn_total_count != g_total_weld_count);
        break;
      case 5U:
        label = "次数";
        snprintf(value, sizeof(value), "%lu", (unsigned long)g_session_weld_count);
        redraw_card = (uint8_t)(redraw_card || g_home_drawn_session_count != g_session_weld_count);
        break;
      default:
        label = "";
        value[0] = '\0';
        break;
    }

    if (redraw_card != 0U)
    {
      if (full_redraw != 0U)
      {
        ST7789_FillRect(box_x, box_y, box_w, box_h, TFT_UI_COLOR_BACKGROUND);
      }
      ST7789_FillRoundRect(box_x, box_y, box_w, box_h, TFT_HOME_CARD_RADIUS, block_colors[i]);
      ST7789_DrawCenteredString(box_x + 4U, box_y + 17U, box_w - 8U, label,
                                TFT_UI_COLOR_CARD_LABEL, block_colors[i]);
      ST7789_DrawCenteredString(box_x + 4U, box_y + 49U, box_w - 8U, value,
                                TFT_UI_COLOR_TEXT, block_colors[i]);
    }
  }

  if ((settings.capacitor_series > 0U) &&
      ((full_redraw != 0U) ||
       (balance_mask != g_home_drawn_balance_mask) ||
       (strcmp(cell_values[0], g_home_drawn_cell_values[0]) != 0)))
  {
    uint16_t fg_color = ((balance_mask & 0x01U) != 0U) ?
                        balance_colors[0] : TFT_UI_COLOR_CARD_LABEL;
    ST7789_DrawString(cell_positions_x[0], cell_positions_y[0], cell_values[0],
                      fg_color, TFT_UI_COLOR_BACKGROUND);
    (void)strncpy(g_home_drawn_cell_values[0], cell_values[0], sizeof(g_home_drawn_cell_values[0]) - 1U);
    g_home_drawn_cell_values[0][sizeof(g_home_drawn_cell_values[0]) - 1U] = '\0';
  }
  if ((settings.capacitor_series > 1U) &&
      ((full_redraw != 0U) ||
       (balance_mask != g_home_drawn_balance_mask) ||
       (strcmp(cell_values[1], g_home_drawn_cell_values[1]) != 0)))
  {
    uint16_t fg_color = ((balance_mask & 0x02U) != 0U) ?
                        balance_colors[1] : TFT_UI_COLOR_CARD_LABEL;
    ST7789_DrawString(cell_positions_x[1], cell_positions_y[1], cell_values[1],
                      fg_color, TFT_UI_COLOR_BACKGROUND);
    (void)strncpy(g_home_drawn_cell_values[1], cell_values[1], sizeof(g_home_drawn_cell_values[1]) - 1U);
    g_home_drawn_cell_values[1][sizeof(g_home_drawn_cell_values[1]) - 1U] = '\0';
  }
  if ((settings.capacitor_series > 2U) &&
      ((full_redraw != 0U) ||
       (balance_mask != g_home_drawn_balance_mask) ||
       (strcmp(cell_values[2], g_home_drawn_cell_values[2]) != 0)))
  {
    uint16_t fg_color = ((balance_mask & 0x04U) != 0U) ?
                        balance_colors[2] : TFT_UI_COLOR_CARD_LABEL;
    ST7789_DrawString(cell_positions_x[2], cell_positions_y[2], cell_values[2],
                      fg_color, TFT_UI_COLOR_BACKGROUND);
    (void)strncpy(g_home_drawn_cell_values[2], cell_values[2], sizeof(g_home_drawn_cell_values[2]) - 1U);
    g_home_drawn_cell_values[2][sizeof(g_home_drawn_cell_values[2]) - 1U] = '\0';
  }
  if ((settings.capacitor_series > 3U) &&
      ((full_redraw != 0U) ||
       (balance_mask != g_home_drawn_balance_mask) ||
       (strcmp(cell_values[3], g_home_drawn_cell_values[3]) != 0)))
  {
    uint16_t fg_color = ((balance_mask & 0x08U) != 0U) ?
                        balance_colors[3] : TFT_UI_COLOR_CARD_LABEL;
    ST7789_DrawString(cell_positions_x[3], cell_positions_y[3], cell_values[3],
                      fg_color, TFT_UI_COLOR_BACKGROUND);
    (void)strncpy(g_home_drawn_cell_values[3], cell_values[3], sizeof(g_home_drawn_cell_values[3]) - 1U);
    g_home_drawn_cell_values[3][sizeof(g_home_drawn_cell_values[3]) - 1U] = '\0';
  }
  if ((full_redraw != 0U) || (strcmp(total_value, g_home_drawn_total_value) != 0))
  {
    ST7789_DrawString(44U, TFT_HOME_FOOTER_TOP + 48U, total_value,
                      TFT_UI_COLOR_CARD_LABEL, TFT_UI_COLOR_BACKGROUND);
    (void)strncpy(g_home_drawn_total_value, total_value, sizeof(g_home_drawn_total_value) - 1U);
    g_home_drawn_total_value[sizeof(g_home_drawn_total_value) - 1U] = '\0';
  }
  if ((full_redraw != 0U) || (charging_active != g_home_drawn_charging))
  {
    uint16_t power_color = (charging_active != 0U) ?
                           TFT_UI_COLOR_CHARGING : TFT_UI_COLOR_CARD_LABEL;
    ST7789_DrawString(124U, TFT_HOME_FOOTER_TOP + 48U, "功率",
                      power_color, TFT_UI_COLOR_BACKGROUND);
    ST7789_DrawString(204U, TFT_HOME_FOOTER_TOP + 48U, "W",
                      power_color, TFT_UI_COLOR_BACKGROUND);
  }
  if ((full_redraw != 0U) || (charging_active != g_home_drawn_charging) ||
      (strcmp(power_value, g_home_drawn_power_value) != 0))
  {
    uint16_t power_color = (charging_active != 0U) ?
                           TFT_UI_COLOR_CHARGING : TFT_UI_COLOR_CARD_LABEL;
    ST7789_DrawString(156U, TFT_HOME_FOOTER_TOP + 48U, power_value,
                      power_color, TFT_UI_COLOR_BACKGROUND);
    (void)strncpy(g_home_drawn_power_value, power_value, sizeof(g_home_drawn_power_value) - 1U);
    g_home_drawn_power_value[sizeof(g_home_drawn_power_value) - 1U] = '\0';
  }
  {
    char energy_value[16];
    if (g_weld_energy_valid != 0U)
    {
      uint32_t energy_dj = (g_weld_energy_j >= 999.9f) ?
                           9999U : (uint32_t)(g_weld_energy_j * 10.0f + 0.5f);
      snprintf(energy_value, sizeof(energy_value), "%lu.%luJ",
               (unsigned long)(energy_dj / 10U),
               (unsigned long)(energy_dj % 10U));
    }
    else
    {
      snprintf(energy_value, sizeof(energy_value), "--.-J");
    }
    ST7789_FillRect(260U, TFT_HOME_FOOTER_TOP + 46U, 60U, 18U,
                    TFT_UI_COLOR_BACKGROUND);
    ST7789_DrawString(260U, TFT_HOME_FOOTER_TOP + 48U, energy_value,
                      TFT_UI_COLOR_CARD_LABEL, TFT_UI_COLOR_BACKGROUND);
  }

  TftUi_DrawMessage();
  ST7789_EndFrame();

  g_home_drawn_settings = g_settings;
  g_home_drawn_total_count = g_total_weld_count;
  g_home_drawn_session_count = g_session_weld_count;
  g_home_drawn_balance_mask = balance_mask;
  g_home_drawn_charging = charging_active;
  g_last_home_redraw = 1;
}

/* 绘制设置菜单及当前选项值。 */
void TftUi_DrawMenu(void)
{
  uint8_t visible_i = 0U;
  uint8_t first_index = 0U;
  const uint8_t visible_items = 7U;
  const uint16_t list_top = 4U;
  const uint16_t row_h = 33U;
  char value[32];
  const uint8_t selected_index = g_menu_index;

  if (selected_index >= visible_items)
  {
    first_index = (uint8_t)(selected_index - visible_items + 1U);
  }

  ST7789_BeginFrame(TFT_UI_COLOR_BACKGROUND);
  for (visible_i = 0U; visible_i < visible_items; visible_i++)
  {
    uint8_t menu_index = (uint8_t)(first_index + visible_i);
    uint16_t row_y = (uint16_t)(list_top + visible_i * row_h);
    uint16_t bg_color = TFT_UI_COLOR_ROW;
    uint16_t text_color = TFT_UI_COLOR_MENU_TEXT;
    uint8_t row_selected = (uint8_t)(menu_index == selected_index);

    if (row_selected != 0U && g_edit_mode == 0U)
    {
      bg_color = TFT_UI_COLOR_SELECTED;
      text_color = TFT_UI_RGB565(13U, 22U, 38U);
    }

    switch (menu_index)
    {
      case 0U:
        snprintf(value, sizeof(value), "%u ms", (unsigned int)g_settings.pulse1_ms);
        break;
      case 1U:
        snprintf(value, sizeof(value), "%u ms", (unsigned int)g_settings.pulse2_ms);
        break;
      case 2U:
        snprintf(value, sizeof(value), "%u ms", (unsigned int)g_settings.interval_ms);
        break;
      case 3U:
        snprintf(value, sizeof(value), "%u.%u s",
                 (unsigned int)(g_settings.trigger_time_100ms / 10U),
                 (unsigned int)(g_settings.trigger_time_100ms % 10U));
        break;
      case 4U:
        snprintf(value, sizeof(value), "%u 串", (unsigned int)g_settings.capacitor_series);
        break;
      case 5U:
      {
        uint32_t cell_voltage_cv =
            (uint32_t)(g_settings.cell_voltage * 100.0f + 0.5f);
        snprintf(value, sizeof(value), "%lu.%02lu V",
                 (unsigned long)(cell_voltage_cv / 100U),
                 (unsigned long)(cell_voltage_cv % 100U));
        break;
      }
      case 6U:
      {
        uint32_t balance_delta_mv =
            (uint32_t)(g_settings.balance_delta_v * 1000.0f + 0.5f);
        snprintf(value, sizeof(value), "%lu.%03lu V",
                 (unsigned long)(balance_delta_mv / 1000U),
                 (unsigned long)(balance_delta_mv % 1000U));
        break;
      }
      case 7U:
      {
        uint32_t charge_current_da =
            (uint32_t)(g_settings.charge_current_a * 10.0f + 0.5f);
        snprintf(value, sizeof(value), "%lu.%lu A",
                 (unsigned long)(charge_current_da / 10U),
                 (unsigned long)(charge_current_da % 10U));
        break;
      }
      case 8U:
        snprintf(value, sizeof(value), "%s", g_settings.charge_enable != 0U ? "开" : "关");
        break;
      case 9U:
        snprintf(value, sizeof(value), "%u %%", (unsigned int)g_settings.brightness_percent);
        break;
      default:
        value[0] = '\0';
        break;
    }

    ST7789_FillRect(0U, row_y, ST7789_WIDTH, row_h, bg_color);
    ST7789_DrawString(8U, row_y + 8U, g_menu_names[menu_index],
                      text_color, bg_color);
    if (value[0] != '\0')
    {
      uint16_t value_width = ST7789_GetStringWidth(value);
      uint16_t value_x = (uint16_t)(ST7789_WIDTH - 8U - value_width);
      uint16_t value_fg = TFT_UI_COLOR_MENU_TEXT;
      uint16_t value_bg = bg_color;

      if (row_selected != 0U && g_edit_mode != 0U)
      {
        uint16_t highlight_x = (value_x >= 2U) ? (uint16_t)(value_x - 2U) : 0U;
        ST7789_FillRect(highlight_x, row_y, (uint16_t)(value_width + 4U),
                        row_h, TFT_UI_COLOR_EDIT);
        value_fg = TFT_UI_COLOR_BACKGROUND;
        value_bg = TFT_UI_COLOR_EDIT;
      }

      ST7789_DrawString(value_x, row_y + 8U, value, value_fg, value_bg);
    }
  }

  TftUi_DrawMessage();
  ST7789_EndFrame();
  g_menu_needs_full_redraw = 0U;
}

/* 绘制校准选项及已保存的校准数据。 */
static void TftUi_DrawCalibration(void)
{
  static const char * const choices[3U] =
  {
    "容量校准",
    "内阻基准校准",
    "返回"
  };
  TftUiCalibrationData_t calibration_data = {0};
  char capacity_text[32];
  char voltage_drop_text[32];
  uint8_t i = 0U;

  TftUi_LoadCalibrationData(&calibration_data);
  if (calibration_data.capacitance_f > 0.0f)
  {
    uint32_t capacity_tenth =
      (uint32_t)(calibration_data.capacitance_f * 10.0f + 0.5f);
    snprintf(capacity_text, sizeof(capacity_text), "容量: %lu.%luF",
             (unsigned long)(capacity_tenth / 10U),
             (unsigned long)(capacity_tenth % 10U));
  }
  else
  {
    snprintf(capacity_text, sizeof(capacity_text), "容量: --.-F");
  }

  if (calibration_data.voltage_drop_v > 0.0f)
  {
    uint32_t voltage_drop_mv =
      (uint32_t)(calibration_data.voltage_drop_v * 1000.0f + 0.5f);
    snprintf(voltage_drop_text, sizeof(voltage_drop_text), "内阻: %lu.%03luV",
             (unsigned long)(voltage_drop_mv / 1000U),
             (unsigned long)(voltage_drop_mv % 1000U));
  }
  else
  {
    snprintf(voltage_drop_text, sizeof(voltage_drop_text), "内阻: --.---V");
  }

  ST7789_BeginFrame(TFT_UI_COLOR_BACKGROUND);
  for (i = 0U; i < 3U; i++)
  {
    const uint16_t row_height = 40U;
    const uint16_t row_gap = 4U;
    const uint16_t list_height = 3U * row_height + 2U * row_gap;
    uint16_t y = (uint16_t)((ST7789_HEIGHT - list_height) / 2U - 18U +
                            i * (row_height + row_gap));
    uint16_t text_width = ST7789_GetStringWidth(choices[i]);
    uint16_t text_x = (uint16_t)((ST7789_WIDTH - text_width) / 2U);
    uint16_t bg = (i == g_calibration_selection) ?
                  TFT_UI_COLOR_SELECTED : TFT_UI_COLOR_ROW;
    uint16_t fg = (i == g_calibration_selection) ?
                  TFT_UI_COLOR_BACKGROUND : TFT_UI_COLOR_MENU_TEXT;
    ST7789_FillRect(0U, y, ST7789_WIDTH, row_height, bg);
    ST7789_DrawString(text_x, y + 12U, choices[i], fg, bg);
  }
  ST7789_DrawCenteredString(8U, 190U, ST7789_WIDTH - 16U, capacity_text,
                            TFT_UI_COLOR_MENU_TEXT, TFT_UI_COLOR_BACKGROUND);
  ST7789_DrawCenteredString(8U, 211U, ST7789_WIDTH - 16U, voltage_drop_text,
                            TFT_UI_COLOR_MENU_TEXT, TFT_UI_COLOR_BACKGROUND);
  TftUi_DrawMessage();
  ST7789_EndFrame();
}

/* 在当前页面上优先叠加持续安全提示，否则显示限时提示。 */
static void TftUi_DrawMessage(void)
{
  const char *message = NULL;
  uint16_t bg = TFT_UI_COLOR_MESSAGE_BACKGROUND;
  uint16_t fg = TFT_UI_COLOR_MESSAGE_TEXT;

  if ((g_safety_alert_mask & (uint8_t)(1U << TFT_UI_SAFETY_ALERT_CONTROL)) != 0U)
  {
    message = g_safety_alert_messages[TFT_UI_SAFETY_ALERT_CONTROL];
  }
  else if (g_msg[0] != '\0')
  {
    message = g_msg;
  }

  if (message != NULL && message[0] != '\0')
  {
    const uint16_t box_w = 260U;
    const uint16_t box_h = 44U;
    const uint16_t box_x = (ST7789_WIDTH - box_w) / 2U;
    const uint16_t box_y = (ST7789_HEIGHT - box_h) / 2U;
    if (g_safety_alert_mask != 0U)
    {
      bg = TFT_UI_COLOR_ALERT_BACKGROUND;
      fg = TFT_UI_COLOR_ALERT_TEXT;
    }
    ST7789_FillRoundRect(box_x, box_y, box_w, box_h, 6U, bg);
    ST7789_DrawCenteredString(box_x + 4U, box_y + 13U, box_w - 8U, message, fg, bg);
  }
}

/* 根据当前页面绘制主体内容和提示消息。 */
void TftUi_RefreshDisplay(void)
{
  if (g_screen == TFT_UI_SCREEN_HOME)
  {
    TftUi_DrawHome();
  }
  else if (g_screen == TFT_UI_SCREEN_MENU)
  {
    TftUi_DrawMenu();
  }
  else
  {
    TftUi_DrawCalibration();
  }
}

/* 维护消息和首页刷新时序，并处理待刷新的界面。 */
void TftUi_Task(void)
{
  uint32_t now = HAL_GetTick();

  ST7789_Task();
  if (ST7789_FrameIsBusy() != 0U)
  {
    return;
  }

  if (BootSplash_IsActive() != 0U)
  {
    return;
  }

  if (g_msg[0] != '\0' && (int32_t)(HAL_GetTick() - g_msg_deadline_ms) >= 0)
  {
    g_msg[0] = '\0';
    g_msg_deadline_ms = 0;
    g_menu_needs_full_redraw = 1U;
    g_dirty = 1;
  }

  if ((g_screen == TFT_UI_SCREEN_HOME) &&
      ((uint32_t)(now - g_home_adc_refresh_tick_ms) >= TFT_HOME_ADC_REFRESH_MS))
  {
    g_home_adc_refresh_tick_ms = now;
    g_dirty = 1U;
  }

  if (g_menu_index >= TFT_UI_MENU_ITEMS)
  {
    g_menu_index = 0;
    g_dirty = 1;
  }

  if (g_dirty)
  {
    TftUi_RefreshDisplay();
    g_dirty = 0;
  }
}
