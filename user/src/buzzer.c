#include "buzzer.h"

/*
 * buzzer.c
 * ------------------------------------------------------------
 * 蜂鸣器非阻塞控制实现。
 *
 * 这里使用一个截止时间戳和一个布尔状态位来表示鸣叫时长，
 * 而不是使用 HAL_Delay() 做忙等待。这样主循环可以继续执行：
 *  - 扫描编码器；
 *  - 更新 TFT 界面；
 *  - 处理充电与保护逻辑；
 *  - 监控各类状态切换。
 */

static uint32_t g_buzzer_deadline_ms = 0;
static uint8_t g_buzzer_active = 0;

/* 蜂鸣器不使用 HAL_Delay，避免提示音阻塞控制和界面任务。 */
/* 初始化蜂鸣器输出和非阻塞提示音状态。 */
void Buzzer_Init(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  g_buzzer_deadline_ms = 0;
  g_buzzer_active = 0;
}

/*
 * Buzzer_Off
 * ------------------------------------------------------------
 * 直接关闭蜂鸣器输出，并清空所有与铃声计时相关的状态。
 *
 * 这一逻辑在多种场景中都会用到：
 *   - 用户确认操作时停止当前提示；
 *   - 报警/错误状态被清除时停掉声音；
 *   - 电源管理切换到安全状态时强制静音。
 *
 * 它会同时清理 g_buzzer_active 和 g_buzzer_deadline_ms，从而避免
 * 后续 Buzzer_Task() 继续处理已经失效的截止时间。
 */
void Buzzer_Off(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  g_buzzer_active = 0;
  g_buzzer_deadline_ms = 0;
}

/*
 * Buzzer_On
 * ------------------------------------------------------------
 * 将蜂鸣器输出置高，进入“鸣叫”状态。
 *
 * 与 Buzzer_Beep() 不同的是，这个接口不设置计时器；它适合用于：
 *   - 持续鸣叫的报警状态；
 *   - 需要手动保持蜂鸣器开启的场景；
 *   - 在某些控制逻辑中，蜂鸣器被视为持续状态灯。
 *
 * 完成输出后，g_buzzer_active 会被设为 1，以确保后续的
 * Buzzer_Task() 能根据现有状态检查是否需要在到时自动关闭。
 */
void Buzzer_On(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  g_buzzer_active = 1;
}

/*
 * Buzzer_Beep
 * ------------------------------------------------------------
 * 启动指定时长的非阻塞蜂鸣提示。
 *
 * 这是本模块最关键的接口：
 *   - 先判断是否传入了合法时长；
 *   - 若为 0，则视为立即关闭；
 *   - 若大于 0，则记录截至时间（HAL_GetTick() + ms），并立即开启
 *     蜂鸣器，让控制流程立即响应。
 *
 * 因为实际关闭动作并不在此函数中执行，而是在 Buzzer_Task() 中
 * 检测到时钟超过 deadline 后调用 Buzzer_Off()，所以它不会阻塞主
 * 循环，也不会干扰实时任务和屏幕更新。
 */
void Buzzer_Beep(uint16_t ms)
{
  if (ms == 0)
  {
    Buzzer_Off();
    return;
  }

  g_buzzer_deadline_ms = HAL_GetTick() + ms;
  Buzzer_On();
}

/*
 * Buzzer_Task
 * ------------------------------------------------------------
 * 主循环周期性调用，用于检查是否到达蜂鸣截止时间。
 *
 * 运行逻辑非常简单：
 *   - 如果当前未处于鸣叫状态，直接返回；
 *   - 若存在一个有效的截止时间且当前时间已经超过该时间，
 *     则调用 Buzzer_Off() 关闭蜂鸣器；
 *   - 否则保持当前状态不变。
 *
 * 这样就把“时间到关闭蜂鸣器”的动作从用户调用处剥离出来，
 * 让调用方无需自己管理等待和超时，整体更稳定也更易维护。
 */
void Buzzer_Task(void)
{
  if (g_buzzer_active == 0)
  {
    return;
  }

  if (g_buzzer_deadline_ms != 0 && (int32_t)(HAL_GetTick() - g_buzzer_deadline_ms) >= 0)
  {
    Buzzer_Off();
  }
}
