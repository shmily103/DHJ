#include "ch224a.h"
#include "i2c.h"

/*
 * ch224a.c
 * ------------------------------------------------------------
 * CH224A 协议控制器实现：用于探测并驱动 USB-PD / PPS 供电端的
 * 状态协商。
 *
 * 在主系统中，CH224A 主要负责两件事：
 *   1. 检测外部电源是否存在并识别其地址；
 *   2. 按系统要求发送 PPS 电压请求，以适配不同输入源和负载。
 * 这些动作都通过 I2C 访问芯片寄存器实现，隐藏了细节，避免上层
 * 代码被协议位域与寄存器布局污染。
 */

#define CH224A_VOLTAGE_PPS 6U

ch224a_device_t ch224a_dev = {0};
static uint8_t g_pps_mode_active = 0U;

/*
 * CH224A_ReadRegister8
 * ------------------------------------------------------------
 * 读取 CH224A 的 8 位寄存器值。
 *
 * 这是底层协议访问的最小单元，负责执行：
 *   - 参数合法性检查；
 *   - 通过 I2C Mem Read 读取寄存器；
 *   - 若失败则调用 I2C1_RecoverBus() 恢复总线状态。
 *
 * 因为上层逻辑不需要关心寄存器细节，所以这里把所有协议错误
 * 处理统一收敛，避免其余模块出现重复的 I2C 恢复逻辑。
 */
static uint8_t CH224A_ReadRegister8(uint8_t addr, uint8_t reg, uint8_t *value)
{
  if (value == NULL)
  {
    return HAL_ERROR;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)((uint16_t)addr << 1U),
                       reg, I2C_MEMADD_SIZE_8BIT, value, 1U, 50U) != HAL_OK)
  {
    (void)I2C1_RecoverBus();
    return HAL_ERROR;
  }
  return HAL_OK;
}

/*
 * CH224A_WriteRegister8
 * ------------------------------------------------------------
 * 向 CH224A 的 8 位寄存器写入一个字节。
 *
 * 这个函数用于：
 *   - 请求电源协议控制字；
 *   - 发送 PPS 目标电压设置；
 *   - 控制协议状态转换。
 *
 * 写入失败时不尝试“猜测”是否成功，而是直接恢复 I2C 总线并
 * 返回 HAL_ERROR；这样上层模块能依据返回值决定是否重试或中止
 * 协商流程，避免错误状态被误当作成功。
 */
static uint8_t CH224A_WriteRegister8(uint8_t addr, uint8_t reg, uint8_t value)
{
  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)((uint16_t)addr << 1U),
                        reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 50U) != HAL_OK)
  {
    (void)I2C1_RecoverBus();
    return HAL_ERROR;
  }
  return HAL_OK;
}

/*
 * CH224A_Init
 * ------------------------------------------------------------
 * 初始化 CH224A 设备状态对象。
 *
 * 此函数会：
 *   - 清空 ch224a_dev 的所有状态；
 *   - 设置默认地址为 CH224A_ADDR_1；
 *   - 把设备状态重置为 DISCONNECTED；
 *   - 清除 PPS 模式标志位。
 *
 * 它不执行真正的 I2C 设备探测，而是为后续 DetectDevice() 做准备，
 * 让上层在首次启动时能安全地重新初始化协议状态。
 */
void CH224A_Init(void)
{
  memset(&ch224a_dev, 0, sizeof(ch224a_dev));
  ch224a_dev.i2c_addr = CH224A_ADDR_1;
  ch224a_dev.state = DEVICE_STATE_DISCONNECTED;
  g_pps_mode_active = 0U;
}

/*
 * CH224A_IsConnected
 * ------------------------------------------------------------
 * 返回当前 CH224A 是否已确认连接。
 *
 * 这里不是直接读取硬件状态，而是返回内部状态对象中的状态值。
 * 因为连接状态并不总是由一次唯一的探测完成：设备可能因总线问题
 * 或电源不稳而失联，所以使用一个中间层状态更适合统一管理。
 */
uint8_t CH224A_IsConnected(void)
{
  return (uint8_t)(ch224a_dev.state != DEVICE_STATE_DISCONNECTED);
}

/*
 * CH224A_DetectDevice
 * ------------------------------------------------------------
 * 扫描两条潜在的 CH224A I2C 地址，确认实际设备所在地址。
 *
 * 逻辑：
 *   - 依次尝试 0x22 和 0x23；
 *   - 任一地址响应状态寄存器成功，则认为设备存在；
 *   - 若两者都失败，则清空状态，并回退到 DISCONNECTED。
 *
 * 成功后，状态寄存器中的 protocol_status 被保存，并视协议状态
 * 是否处于非 PPS 模式决定是否关闭 g_pps_mode_active。这个步骤能
 * 让后继的电压请求逻辑知道当前是否已经在特定供电协商模式中。
 */
uint8_t CH224A_DetectDevice(void)
{
  uint8_t status = 0U;

  if (CH224A_ReadRegister8(CH224A_ADDR_1, CH224A_REG_STATUS, &status) == HAL_OK)
  {
    ch224a_dev.i2c_addr = CH224A_ADDR_1;
  }
  else if (CH224A_ReadRegister8(CH224A_ADDR_2, CH224A_REG_STATUS, &status) == HAL_OK)
  {
    ch224a_dev.i2c_addr = CH224A_ADDR_2;
  }
  else
  {
    ch224a_dev.state = DEVICE_STATE_DISCONNECTED;
    ch224a_dev.protocol_status = 0U;
    g_pps_mode_active = 0U;
    return 0U;
  }

  ch224a_dev.protocol_status = status;
  if ((status & 0x18U) == 0U)
  {
    g_pps_mode_active = 0U;
  }
  ch224a_dev.state = DEVICE_STATE_CONNECTED;
  return 1U;
}

/*
 * CH224A_ReadPdoMaxCurrentMa
 * ------------------------------------------------------------
 * 从 CH224A 读取当前可用 PDO 的最大输出电流。
 *
 * 这个功能在电源管理中相当关键：
 *   - 先确认设备仍在线；
 *   - 读取状态寄存器判断当前是否处于可用的协议状态；
 *   - 再读取 PDO 电流寄存器；
 *   - 将寄存器中的单位值转换成 mA，并返回给上层。
 *
 * 如果设备未进入可用状态、寄存器为空或 I2C 调用失败，则返回
 * HAL_ERROR，并把 *current_ma 置 0，避免上层拿到无效、老数据。
 */
HAL_StatusTypeDef CH224A_ReadPdoMaxCurrentMa(uint16_t *current_ma)
{
  uint8_t status = 0U;
  uint8_t current_units = 0U;

  if (current_ma == NULL)
  {
    return HAL_ERROR;
  }

  if ((CH224A_ReadRegister8(ch224a_dev.i2c_addr, CH224A_REG_STATUS,
                            &status) != HAL_OK) ||
      ((status & 0x18U) == 0U) ||
      (CH224A_ReadRegister8(ch224a_dev.i2c_addr, CH224A_REG_PDO_CURRENT,
                            &current_units) != HAL_OK) ||
      (current_units == 0U))
  {
    *current_ma = 0U;
    return HAL_ERROR;
  }

  ch224a_dev.protocol_status = status;
  *current_ma = (uint16_t)current_units * CH224A_PDO_CURRENT_LSB_MA;
  return HAL_OK;
}

HAL_StatusTypeDef CH224A_ReadPowerData(uint8_t *data)
{
  uint8_t i = 0U;

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < CH224A_POWER_DATA_SIZE; i++)
  {
    if (CH224A_ReadRegister8(ch224a_dev.i2c_addr,
                             (uint8_t)(CH224A_REG_POWER_DATA_START + i),
                             &data[i]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/*
 * CH224A_RequestPPSVoltage
 * ------------------------------------------------------------
 * 向 CH224A 请求指定的 PPS 电压。
 *
 * 该函数会先确保电压值属于合法范围：
 *   - 最低 3300 mV；
 *   - 最高 12000 mV；
 *   - 必须为 100 mV 的整数倍。
 *
 * 之后它会：
 *   1. 将电压转换为 100 mV 单位的寄存器值；
 *   2. 向 PPS 电压寄存器写入目标值；
 *   3. 若还未进入 PPS 模式，则额外写入电压控制寄存器，反映
 *      “使用 PPS 协议”模式。
 *
 * 如果任何一步失败，函数返回 HAL_ERROR，调用者可决定重试或中止
 * 充电/协商流程；这样能避免错误需求继续驱动电源端。
 */
HAL_StatusTypeDef CH224A_RequestPPSVoltage(uint8_t addr, uint16_t voltage_mv)
{
  uint8_t voltage_100mv = 0U;

  if ((voltage_mv < CH224A_PPS_MIN_VOLTAGE_MV) ||
      (voltage_mv > CH224A_PPS_MAX_VOLTAGE_MV) ||
      ((voltage_mv % 100U) != 0U))
  {
    return HAL_ERROR;
  }

  voltage_100mv = (uint8_t)(voltage_mv / 100U);
  if (CH224A_WriteRegister8(addr, CH224A_REG_PPS_VOLTAGE,
                            voltage_100mv) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if ((g_pps_mode_active == 0U) &&
      (CH224A_WriteRegister8(addr, CH224A_REG_VOLTAGE_CTRL,
                             CH224A_VOLTAGE_PPS) != HAL_OK))
  {
    return HAL_ERROR;
  }

  g_pps_mode_active = 1U;
  ch224a_dev.state = DEVICE_STATE_NEGOTIATING;
  return HAL_OK;
}
