#ifndef USER_INC_BOOT_SPLASH_H_
#define USER_INC_BOOT_SPLASH_H_

#include <stdint.h>

/*
 * boot_splash.h
 * ------------------------------------------------------------
 * 提供非阻塞启动画面接口。调用 BootSplash_Show() 提交画面后，
 * 主循环可通过 BootSplash_IsActive() 判断何时切换到正常界面。
 */

/* 显示启动画面并开始计时；实际停留时间从整帧传输完成后开始。 */
void BootSplash_Show(void);
uint8_t BootSplash_IsActive(void);

#endif /* USER_INC_BOOT_SPLASH_H_ */
