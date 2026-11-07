#include "ina226.h"
#include "i2c.h"

/*
 * ina226.c
 * ------------------------------------------------------------
 * INA226 监控实现：负责完成总线探测、寄存器配置和换算逻辑。
 *
 * 这里把物理层的 I2C 访问和应用层的工程换算分离：
 *   - 原始寄存器读取与写入只负责传输；
 *   - 计算器件读取值到电流、电压和功率时，才按手册规定转换。
 * 同时保留设备连接状态，以便上层在采样前先判断芯片是否在线。
 */

/* 保存最近一次初始化/探测的结果；成功初始化后由读取接口用于判断传感器是否可用。 */
static uint8_t g_ina226_connected = 0U;

/* 电流寄存器每个计数对应的安培数；头文件中的配置以微安为单位，故除以 10^6。 */
static const float g_current_lsb_a = (float)INA226_CURRENT_LSB_UA / 1000000.0f;

/* INA226 功率寄存器每个计数对应的瓦特数，数据手册规定为 25 倍电流 LSB。 */
static const float g_power_lsb_w = 25.0f * (float)INA226_CURRENT_LSB_UA / 1000000.0f;

/*
 * INA226 的 16 位寄存器按高字节在前传输。
 * 该内部辅助函数只负责 I2C 读出和拼接原始值，不进行物理单位换算。
 */
static HAL_StatusTypeDef INA226_ReadReg16Raw(uint8_t reg, uint16_t *value)
{
  /* INA226 每个寄存器占两个字节；先清零，便于避免未初始化数据参与拼接。 */
  uint8_t raw[2] = {0U, 0U};

  /* 输出指针为空时无法返回读数，按 HAL 约定报告参数错误。 */
  if (value == NULL)
  {
    return HAL_ERROR;
  }

  /* 使用 I2C1 读取指定的 8 位寄存器地址；超时时间为 50 ms。 */
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
      &hi2c1, INA226_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, raw, 2U, 50U);
  if (result != HAL_OK)
  {
    /* 读操作失败时尝试恢复 I2C1 总线，但保留并返回本次传输的原始错误码。 */
    (void)I2C1_RecoverBus();
    return result;
  }

  /* INA226 以高字节优先返回数据，将两个字节组合成无符号 16 位寄存器值。 */
  *value = (uint16_t)((uint16_t)raw[0] << 8U) | (uint16_t)raw[1];
  return HAL_OK;
}

/*
 * 按高字节优先格式写入 INA226 的 16 位寄存器。
 * 返回值保留 HAL_StatusTypeDef 的数值表示，以兼容公开接口的 uint8_t 返回类型。
 */
static uint8_t INA226_WriteReg16Raw(uint8_t reg, uint16_t value)
{
  /* 发送缓冲区依照 INA226 的高字节在前格式排列。 */
  uint8_t raw[2];
  HAL_StatusTypeDef result = HAL_OK;
  raw[0] = (uint8_t)((value >> 8U) & 0xFFU);
  raw[1] = (uint8_t)(value & 0xFFU);

  /* 通过 I2C1 向指定寄存器写入两个字节，超时时间为 50 ms。 */
  result = HAL_I2C_Mem_Write(&hi2c1, INA226_ADDRESS, reg,
                             I2C_MEMADD_SIZE_8BIT, raw, 2U, 50U);
  if (result != HAL_OK)
  {
    /* 写操作失败后尝试恢复总线；调用方仍会收到原始写操作状态。 */
    (void)I2C1_RecoverBus();
  }
  return (uint8_t)result;
}

/*
 * 探测并初始化 INA226。
 * 只有地址探测、厂商 ID 校验、配置寄存器写入和校准寄存器写入全部成功，
 * 才将连接状态标记为已连接；任一步骤失败都会清除状态并提前返回。
 */
/*
 * INA226_Init
 * ------------------------------------------------------------
 * 初始化并探测 INA226，确保它确实在线并可用于测量。
 *
 * 该函数按顺序执行：
 *   1. 通过 I2C 地址探测判断 0x40 地址设备是否存在；
 *   2. 读取 manufacturer ID，确认设备确为 TI INA226；
 *   3. 写入默认配置寄存器，设定工作模式及转换参数；
 *   4. 写入校准寄存器，根据 shunt 和电流 LSB 设置换算系数。
 *
 * 若任何一步失败，则会清除 g_ina226_connected，避免系统在后续
 * 读取中认定设备已可用，从而使用无效数据。
 */
void INA226_Init(void)
{
  /* 厂商 ID 读回值用于确认目标设备确为 TI INA226。 */
  uint16_t man_id = 0U;
  /* 使用头文件定义的默认测量配置和校准参数。 */
  uint16_t config = INA226_CONFIG_DEFAULT;
  HAL_StatusTypeDef result = HAL_OK;

  /* 先确认 I2C 总线上存在响应设备；地址宏使用 HAL 所需的地址格式。 */
  result = HAL_I2C_IsDeviceReady(&hi2c1, INA226_ADDRESS, 2U, 50U);
  if (result != HAL_OK)
  {
    g_ina226_connected = 0U;
    return;
  }

  /* 读取芯片厂商 ID。 */
  result = INA226_ReadReg16Raw(INA226_MANUFACTURER_ID_REG, &man_id);
  if (result != HAL_OK)
  {
    g_ina226_connected = 0U;
    return;
  }

  /* TI 的 INA226 厂商 ID 应为 0x5449；不匹配时不对未知设备写配置。 */
  if (man_id != 0x5449U)
  {
    g_ina226_connected = 0U;
    return;
  }

  /* 配置测量模式、转换时间及平均次数等参数。 */
  result = (HAL_StatusTypeDef)INA226_WriteRegister16(INA226_CONFIG_REG, config);
  if (result != HAL_OK)
  {
    g_ina226_connected = 0U;
    return;
  }

  /* 写入校准寄存器；该值与电流 LSB 配置共同决定电流和功率换算结果。 */
  result = (HAL_StatusTypeDef)INA226_WriteRegister16(
      INA226_CALIBRATION_REG, INA226_CALIBRATION_DEFAULT);
  if (result != HAL_OK)
  {
    g_ina226_connected = 0U;
    return;
  }

  /* 所有初始化步骤完成后才公布传感器可用状态。 */
  g_ina226_connected = 1U;
}

/* 返回 INA226 最近一次初始化/探测所记录的连接状态；非零表示初始化成功。 */
/*
 * INA226_IsConnected
 * ------------------------------------------------------------
 * 返回最近一次初始化/探测后的连接状态。
 *
 * 这个状态不是实时读取寄存器，而是保存于静态标志位的缓存值；
 * 它适合作为上层判断“传感器是否在线”的快速接口。
 *
 * 由于 I2C 总线可能偶发失效，因此上层在采样前也常需要使用
 * INA226_ReadMeasurements() 这种返回 HAL 状态的接口做最终确认。
 */
uint8_t INA226_IsConnected(void)
{
  return g_ina226_connected;
}

/*
 * 依次读取母线电压、电流和功率，并将原始寄存器计数换算为 V、A、W。
 * measurements 为空、器件未初始化成功或任一寄存器读取失败时返回 HAL_ERROR。
 */
/*
 * INA226_ReadMeasurements
 * ------------------------------------------------------------
 * 一次性读取并换算 INA226 的总线电压、电流和功率。
 *
 * 处理流程：
 *   - 检查输出参数和初始化状态是否有效；
 *   - 连续读取 BUS_VOLTAGE、CURRENT、POWER 三个寄存器；
 *   - 将原始计数按照 INA226 手册规则换算为 V/A/W；
 *   - 写回到调用者提供的结构体。
 *
 * 这是上层电源管理模块最核心的接口：它避免了每个调用者自己理解
 * INA226 的寄存器格式和补码规则，保证所有测量逻辑保持一致。
 */
HAL_StatusTypeDef INA226_ReadMeasurements(INA226_Measurements_t *measurements)
{
  /* 三个原始寄存器值均先以无单位的 16 位计数保存。 */
  uint16_t voltage_raw = 0U;
  uint16_t current_raw = 0U;
  uint16_t power_raw = 0U;

  /* 组合读取接口要求有效的输出对象，并要求先成功完成设备初始化。 */
  if ((measurements == NULL) || (g_ina226_connected == 0U))
  {
    return HAL_ERROR;
  }

  /* 任意一次 I2C 读取失败即中止，避免向调用方返回不完整的一组测量值。 */
  if ((INA226_ReadReg16Raw(INA226_BUS_VOLTAGE_REG, &voltage_raw) != HAL_OK) ||
      (INA226_ReadReg16Raw(INA226_CURRENT_REG, &current_raw) != HAL_OK) ||
      (INA226_ReadReg16Raw(INA226_POWER_REG, &power_raw) != HAL_OK))
  {
    return HAL_ERROR;
  }

  /*
   * 母线电压寄存器每位对应 1.25 mV（0.00125 V）。
   * 电流寄存器是有符号二进制补码，按当前电流 LSB 换算为安培；
   * 功率寄存器按 INA226 的功率 LSB 换算为瓦特。
   */
  measurements->bus_voltage_v = (float)voltage_raw * 0.00125f;
  measurements->current_a = (float)((int16_t)current_raw) * g_current_lsb_a;
  measurements->power_w = (float)power_raw * g_power_lsb_w;
  return HAL_OK;
}

/*
 * 对外的原始寄存器读取接口。
 * 底层 I2C 读取失败时返回初始化的零值；因此调用方若需区分错误与有效零值，
 * 应优先使用返回 HAL 状态的 INA226_ReadMeasurements 接口。
 */
/*
 * INA226_ReadRegister16
 * ------------------------------------------------------------
 * 对外返回 16 位寄存器值。
 *
 * 这个函数提供了一个简单的原始寄存器读取接口，适合调试、
 * 诊断以及不需要复杂单位换算的场景。
 *
 * 若底层 I2C 失败，函数会返回 0，调用方必须注意：0 既可能是
 * 真值，也可能是错误值，因此更推荐优先使用
 * INA226_ReadMeasurements() 这类带状态返回的接口。
 */
uint16_t INA226_ReadRegister16(uint8_t reg)
{
  /* 默认值也作为底层读取失败时的返回值。 */
  uint16_t value = 0U;
  (void)INA226_ReadReg16Raw(reg, &value);
  return value;
}

/* 对外提供 16 位寄存器写入接口，返回值为 HAL 状态码的 uint8_t 表示。 */
uint8_t INA226_WriteRegister16(uint8_t reg, uint16_t value)
{
  return INA226_WriteReg16Raw(reg, value);
}

/*
 * 读取电流寄存器的有符号原始计数。
 * 电流寄存器采用 16 位二进制补码表示，因此需转换为 int16_t 才能保留方向。
 */
int16_t INA226_ReadCurrentRaw(void)
{
  uint16_t raw = INA226_ReadRegister16(INA226_CURRENT_REG);
  return (int16_t)raw;
}

/* 读取有符号电流计数，并乘以每计数对应的安培数得到实际电流。 */
/*
 * INA226_ReadCurrentA
 * ------------------------------------------------------------
 * 返回当前输入/输出电流，单位为安培。
 *
 * 实现方式很直接：读取当前寄存器的有符号原始计数，再乘以当前
 * LSB 对应的安培数；该值可以为正或负，正值表示流入，负值表示
 * 反向流动。
 *
 * 这是判断电源管理是否处于过流、反向电流或正常充电状态的重要
 * 基础数据之一。
 */
float INA226_ReadCurrentA(void)
{
  return (float)INA226_ReadCurrentRaw() * g_current_lsb_a;
}

/* 读取无符号母线电压计数；每计数为 1.25 mV，结果以伏特返回。 */
float INA226_ReadBusVoltageV(void)
{
  uint16_t raw = INA226_ReadRegister16(INA226_BUS_VOLTAGE_REG);
  return (float)raw * 0.00125f;
}

/*
 * 读取有符号分流电压计数；每计数为 2.5 uV（0.0025 mV），
 * 将原始值解释为 int16_t 后以毫伏返回，以保留分流电压的正负方向。
 */
float INA226_ReadShuntVoltageMv(void)
{
  uint16_t raw = INA226_ReadRegister16(INA226_SHUNT_VOLTAGE_REG);
  return (float)((int16_t)raw) * 0.0025f;
}

/* 读取无符号功率计数，并按配置的功率 LSB 换算为瓦特。 */
float INA226_ReadPowerW(void)
{
  uint16_t raw = INA226_ReadRegister16(INA226_POWER_REG);
  return (float)raw * g_power_lsb_w;
}

/* 读取厂商 ID 寄存器；芯片型号校验使用的期望 TI 标识为 0x5449。 */
uint16_t INA226_ReadManufacturerId(void)
{
  return INA226_ReadRegister16(INA226_MANUFACTURER_ID_REG);
}

/* 读取器件 ID 寄存器，可用于区分或诊断 INA226 芯片型号。 */
uint16_t INA226_ReadDieId(void)
{
  return INA226_ReadRegister16(INA226_DIE_ID_REG);
}
