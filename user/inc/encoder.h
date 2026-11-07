#ifndef USER_INC_ENCODER_H_
#define USER_INC_ENCODER_H_

#include <stdint.h>
#include "main.h"
#include "tim.h"

/*
 * encoder.h
 * ------------------------------------------------------------
 * 旋钮编码器和按键输入层。
 *
 * 系统中的旋钮不仅用于菜单调节，还承担大量参数编辑动作；
 * 因此这里提供了不同速度的旋转事件和按键事件，用户界面可以
 * 根据当前输入模式决定是否执行快速步进、长按确认或点击确认。
 */

/* 编码器旋转、按键短按和长按事件；快速旋转会产生独立事件。 */
typedef enum
{
  ENCODER_EVENT_NONE = 0,
  ENCODER_EVENT_CW,
  ENCODER_EVENT_CCW,
  ENCODER_EVENT_CW_FAST,
  ENCODER_EVENT_CCW_FAST,
  ENCODER_EVENT_CLICK,
  ENCODER_EVENT_LONG_PRESS
} EncoderEvent_t;

/* 在主循环中周期调用 Encoder_Task，再用 Encoder_GetEvent 逐个取出事件。 */
void Encoder_Init(void);
void Encoder_Task(void);
EncoderEvent_t Encoder_GetEvent(void);

#endif /* USER_INC_ENCODER_H_ */
