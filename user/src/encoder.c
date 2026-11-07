#include "encoder.h"
#include "control.h"

/*
 * encoder.c
 * ------------------------------------------------------------
 * 编码器输入处理实现。
 *
 * 该模块负责把 EC11 旋钮和按键信号转换成统一的事件流，
 * 其工作方式如下：
 *   - 定时器计数器持续记录增量；
 *   - 通过 detent 余数把机械卡点转换为方向事件；
 *   - 通过按键按下/释放时长区分短按与长按；
 *   - 最后将事件写入队列，让主循环按顺序消费。
 *
 * 这样 UI 层无需关心底层 GPIO、计时器和中断，
 * 只需调用 Encoder_GetEvent() 即可处理输入。
 */

#define ENCODER_COUNTS_PER_DETENT 4
#define ENCODER_FAST_DETENT_MS 80U  
#define ENCODER_FAST_THRESHOLD 3U
#define ENCODER_EVENT_QUEUE_SIZE 32U
#define ENCODER_LONG_PRESS_MS 500U

static volatile EncoderEvent_t g_encoder_event = ENCODER_EVENT_NONE;
/* 使用环形队列缓存旋转事件；按键事件单独保存并优先交给主循环。 */
static EncoderEvent_t g_encoder_event_queue[ENCODER_EVENT_QUEUE_SIZE];
static uint8_t g_encoder_event_queue_head = 0U;
static uint8_t g_encoder_event_queue_tail = 0U;
static volatile int32_t g_encoder_last_counter = 0;
static int32_t g_encoder_count_remainder = 0;
static uint32_t g_encoder_last_detent_ms = 0U;
static uint8_t g_encoder_has_previous_detent = 0U;
static int8_t g_encoder_last_dir = 0;     
static uint8_t g_encoder_fast_streak = 0U;
static volatile uint32_t g_encoder_button_press_ms = 0;
static volatile uint8_t g_encoder_button_down = 0;

/* 将旋转事件写入环形队列，队列满时丢弃新事件。 */
static void Encoder_QueueRotationEvent(EncoderEvent_t event)
{
  uint8_t next_head = (uint8_t)((g_encoder_event_queue_head + 1U) % ENCODER_EVENT_QUEUE_SIZE);

  if (next_head != g_encoder_event_queue_tail)
  {
    g_encoder_event_queue[g_encoder_event_queue_head] = event;
    g_encoder_event_queue_head = next_head;
  }
}

/*
 * Encoder_ProcessEncoder
 * ------------------------------------------------------------
 * 把定时器编码器增量转成方向和速度事件。
 *
 * 这里的关键在于：
 *   - current_counter 可能环绕，所以需要用差值方式处理；
 *   - delta 不是直接按一格脉冲事件处理，而是累计到余数；
 *   - 每达到一个机械卡点（ENCODER_COUNTS_PER_DETENT）就触发一次
 *     CW/CW_FAST 或 CCW/CCW_FAST 事件。
 *
 * 这使得用户旋转时不会因为脉冲过密导致事件过于频繁，同时还保留
 * 了快速旋转时的区分能力，界面上可以对快速调整做更快的步进。
 */
static void Encoder_ProcessEncoder(void)
{
  uint16_t current_counter = __HAL_TIM_GET_COUNTER(&htim3);
  /* 转为有符号差值以兼容 16 位定时器计数器回绕。 */
  int16_t delta = (int16_t)(current_counter - (uint16_t)g_encoder_last_counter);
  g_encoder_last_counter = current_counter;

  if (delta == 0)
  {
    return;
  }

  /* 定时器每个机械卡点产生多个计数，余数保留到下一次扫描。 */
  g_encoder_count_remainder += delta;
  /* 顺时针处理 */
  while (g_encoder_count_remainder >= ENCODER_COUNTS_PER_DETENT)
  {
    uint32_t now = HAL_GetTick();
    uint8_t is_time_close = (uint8_t)(g_encoder_has_previous_detent != 0U &&
                                      (now - g_encoder_last_detent_ms) < ENCODER_FAST_DETENT_MS);

    /* 如果时间够快，且方向一致，则累加连续计数；否则重新计数 */
    if (is_time_close != 0U && g_encoder_last_dir == 1)
    {
      if (g_encoder_fast_streak < 255U)
      {
        g_encoder_fast_streak++;
      }
    }
    else
    {
      g_encoder_fast_streak = 1U; /* 当前这格算作连续动作的第一格 */
      g_encoder_last_dir = 1;     /* 标记当前方向为顺时针 */
    }

    /* 只有当连续快速格数达到阈值后，才输出 FAST 事件 */
    EncoderEvent_t event = (g_encoder_fast_streak >= ENCODER_FAST_THRESHOLD) ?
                           ENCODER_EVENT_CW_FAST : ENCODER_EVENT_CW;

    Encoder_QueueRotationEvent(event);
    g_encoder_count_remainder -= ENCODER_COUNTS_PER_DETENT;
    g_encoder_last_detent_ms = now;
    g_encoder_has_previous_detent = 1U;
  }
  /* 逆时针处理 */
  while (g_encoder_count_remainder <= -ENCODER_COUNTS_PER_DETENT)
  {
    uint32_t now = HAL_GetTick();
    uint8_t is_time_close = (uint8_t)(g_encoder_has_previous_detent != 0U &&
                                      (now - g_encoder_last_detent_ms) < ENCODER_FAST_DETENT_MS);

    if (is_time_close != 0U && g_encoder_last_dir == -1)
    {
      if (g_encoder_fast_streak < 255U)
      {
        g_encoder_fast_streak++;
      }
    }
    else
    {
      g_encoder_fast_streak = 1U;
      g_encoder_last_dir = -1;     /* 标记当前方向为逆时针 */
    }

    EncoderEvent_t event = (g_encoder_fast_streak >= ENCODER_FAST_THRESHOLD) ?
                           ENCODER_EVENT_CCW_FAST : ENCODER_EVENT_CCW;

    Encoder_QueueRotationEvent(event);
    g_encoder_count_remainder += ENCODER_COUNTS_PER_DETENT;
    g_encoder_last_detent_ms = now;
    g_encoder_has_previous_detent = 1U;
  }
}

/*
 * Encoder_ProcessButtonState
 * ------------------------------------------------------------
 * 检测按键的长按状态，并在达到阈值后生成唯一一次长按事件。
 *
 * 当按键处于 press 状态时，不立刻判断是短按还是长按，而是持续
 * 记录按下时间；若超过 ENCODER_LONG_PRESS_MS，则认为长按，并将
 * g_encoder_event 置为 LONG_PRESS。这个设计确保：
 *   - 短按不会在长按被判定后继续触发；
 *   - 一次按下只会产生一个事件，避免重复。
 */
static void Encoder_ProcessButtonState(void)
{
  if (g_encoder_button_down == 0)
  {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((now - g_encoder_button_press_ms) >= ENCODER_LONG_PRESS_MS)
  {
    /* 长按在按下期间触发一次；松开时不再生成短按事件。 */
    g_encoder_event = ENCODER_EVENT_LONG_PRESS;
    g_encoder_button_down = 0;
  }
}

/*
 * Encoder_Init
 * ------------------------------------------------------------
 * 完成编码器定时器和事件队列的初始化。
 *
 * 具体动作：
 *   - 启动 TIM3 的编码器模式；
 *   - 清当前计数器；
 *   - 重置上一次计数值、余数和时间戳；
 *   - 清空 ring buffer 与按键状态；
 *   - 把事件状态设为 NONE。
 *
 * 这一步保证后续的旋转输入和按键输入都从一致的初始状态开始，
 * 避免开机后马上出现旧残留事件。
 */
void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  g_encoder_last_counter = 0;
  g_encoder_count_remainder = 0;
  g_encoder_last_detent_ms = HAL_GetTick();
  g_encoder_has_previous_detent = 0U;
  g_encoder_event_queue_head = 0U;
  g_encoder_event_queue_tail = 0U;
  g_encoder_event = ENCODER_EVENT_NONE;
}

/*
 * Encoder_Task
 * ------------------------------------------------------------
 * 这是主循环中用于持续处理输入的统一入口。
 *
 * 它会定期：
 *   1. 处理旋转增量，产生方向和速度事件；
 *   2. 检查按钮按下持续时间，判断是否触发长按；
 *
 * 这样上层代码只需在主循环里周期调用 Encoder_Task()，便能自动
 * 累积并准备好输入事件，而不用自己处理底层定时器和中断。
 */
void Encoder_Task(void)
{
  Encoder_ProcessEncoder();
  Encoder_ProcessButtonState();
}

/*
 * Encoder_GetEvent
 * ------------------------------------------------------------
 * 从事件缓冲区中按优先级取出一个待处理事件。
 *
 * 优先级顺序：
 *   1. 直接的按键事件（如 CLICK/LONG_PRESS）；
 *   2. 剩余的旋转事件队列。
 *
 * 这样做可以保证用户操作的即时反馈：
 *   - 例如按键短按应该优先于旋转积压事件；
 *   - 旋转事件只在没有更高优先级输入时才被消费。
 *
 * 返回 ENCODER_EVENT_NONE 表示当前无新事件，调用方应继续等待。
 */
EncoderEvent_t Encoder_GetEvent(void)
{
  /* 先取按键事件，再按顺序取出积压的旋转事件。 */
  EncoderEvent_t event = g_encoder_event;
  if (event != ENCODER_EVENT_NONE)
  {
    g_encoder_event = ENCODER_EVENT_NONE;
    return event;
  }

  if (g_encoder_event_queue_tail != g_encoder_event_queue_head)
  {
    event = g_encoder_event_queue[g_encoder_event_queue_tail];
    g_encoder_event_queue_tail = (uint8_t)((g_encoder_event_queue_tail + 1U) % ENCODER_EVENT_QUEUE_SIZE);
    return event;
  }

  return ENCODER_EVENT_NONE;
}

/*
 * HAL_GPIO_EXTI_Callback
 * ------------------------------------------------------------
 * 处理外部中断触发的 GPIO 事件。
 *
 * 这里同时处理两类事件：
 *   - Touch_Pin：触摸屏输入，交给 Control_HandleTouch() 处理；
 *   - EC11_E_Pin：编码器按键的电平变化。
 *
 * 对于编码器按键：
 *   - 下降沿表示按下，记录开始时间；
 *   - 上升沿表示释放，若在长按门限之前则判定为点击事件；
 *   - 如果按下时间过长，则在 ProcessButtonState 中产生长按事件，
 *     释放时不再重复生成短按事件。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == Touch_Pin)
  {
    Control_HandleTouch();
    return;
  }

  if (GPIO_Pin != EC11_E_Pin)
  {
    return;
  }

  if (HAL_GPIO_ReadPin(EC11_E_GPIO_Port, EC11_E_Pin) == GPIO_PIN_RESET)
  {
    g_encoder_button_down = 1;
    g_encoder_button_press_ms = HAL_GetTick();
  }
  else
  {
    if (g_encoder_button_down == 1)
    {
      uint32_t now = HAL_GetTick();
      if ((now - g_encoder_button_press_ms) < ENCODER_LONG_PRESS_MS)
      {
        g_encoder_event = ENCODER_EVENT_CLICK;
      }
      g_encoder_button_down = 0;
    }
  }
}
