#ifndef USER_INC_ST7789_H_
#define USER_INC_ST7789_H_

#include <stdint.h>
#include "main.h"

/*
 * st7789.h
 * ------------------------------------------------------------
 * ST7789 TFT 显示控制器驱动接口。
 *
 * 这个模块负责把 LCD 控制器底层细节（SPI、窗口设置、DMA、缓存）
 * 封装起来，对上层提供绘图和文本接口。通过命令缓存和分批刷新，
 * 能在不频繁阻塞的情况下更新大面积区域，提升显示体验和响应性。
 */

/* 屏幕逻辑分辨率；像素颜色使用 RGB565。 */
#define ST7789_WIDTH 320
#define ST7789_HEIGHT 240
#define ST7789_COLOR_DEPTH 16

#define Light_yellow     0xFC15u
#define Light_green      0x67FBu
#define Mint_Green       0xB45Fu
#define Light_blue       0xFA8Au
#define Light_purple     0x445Fu
#define Light_pink       0xFEA8u

#define ST7789_COLOR_BLACK 0x0000
#define ST7789_COLOR_WHITE 0xFFFF
#define ST7789_COLOR_YELLOW 0xFFE0
#define ST7789_COLOR_BLUE 0x001F
#define ST7789_COLOR_GREEN 0x07E0
#define ST7789_COLOR_RED 0xF800

void ST7789_Init(void);
void ST7789_Reset(void);
void ST7789_WriteCommand(uint8_t cmd);
void ST7789_WriteData(uint8_t data);
void ST7789_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
/*
 * 帧内绘图命令先缓存，EndFrame 启动分批 DMA 刷新。
 * 帧内 48x48 位图会保留源数据指针，源数据必须在帧传输完成前有效。
 * 需要非阻塞刷新的调用方应在 BeginFrame/EndFrame 之间提交绘图命令；
 * 未处于帧模式时，兼容性的单项绘图接口仍会同步发送。
 */
void ST7789_BeginFrame(uint16_t background_color);
void ST7789_EndFrame(void);
/* 主循环应持续调用；每次提交/处理有限的一段 DMA，不等待整帧传输。 */
void ST7789_Task(void);
uint8_t ST7789_FrameIsBusy(void);
void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
uint8_t ST7789_DrawRleImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            const uint8_t *rle, uint32_t rle_size,
                            const uint16_t *palette, uint8_t palette_size,
                            uint8_t palette_index_bits);
void ST7789_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
void ST7789_FillScreen(uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg_color, uint16_t bg_color);
void ST7789_DrawStringLarge(uint16_t x, uint16_t y, const char *str,
                            uint16_t fg_color, uint16_t bg_color);
void ST7789_DrawBitmap48(uint16_t x, uint16_t y, const uint64_t *rows,
                         uint16_t fg_color, uint16_t bg_color);
uint16_t ST7789_GetStringWidth(const char *str);
uint16_t ST7789_GetLargeStringWidth(const char *str);
void ST7789_DrawCenteredString(uint16_t x, uint16_t y, uint16_t box_w, const char *str, uint16_t fg_color, uint16_t bg_color);
void ST7789_DrawNumber(uint16_t x, uint16_t y, int32_t value, uint16_t fg_color, uint16_t bg_color);

#endif /* USER_INC_ST7789_H_ */
