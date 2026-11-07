#ifndef USER_INC_BUZZER_H_
#define USER_INC_BUZZER_H_

#include <stdint.h>
#include "main.h"

/*
 * buzzer.h
 * ------------------------------------------------------------
 * 蜂鸣器驱动接口：用于给用户提供操作反馈，例如开机提示、
 * 操作确认、报警和错误提示。
 *
 * 关键点：
 *   - 整个模块采用非阻塞设计，避免使用 HAL_Delay 导致主循环卡顿；
 *   - Buzzer_Beep() 仅设置一个截止时间，真正的关闭动作由
 *     Buzzer_Task() 在主循环中执行；
 *   - 这样可以确保界面逻辑、编码器扫描和控制流程仍能持续响应。
 */

/* 非阻塞蜂鸣器：启动提示后由 Buzzer_Task 到时自动关闭。 */
void Buzzer_Init(void);
void Buzzer_Off(void);
void Buzzer_On(void);
void Buzzer_Beep(uint16_t ms);
void Buzzer_Task(void);

#endif /* USER_INC_BUZZER_H_ */
