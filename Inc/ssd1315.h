#ifndef __SSD1315_H
#define __SSD1315_H

#include "i2c.h"

void OLED_WriteCmd(uint8_t cmd);
void SSD1315_Init(void);
void SSD1315_Clear(void);
void SSD1315_Update(void);
void SSD1315_DrawPoint(uint8_t x, uint8_t y, uint8_t color);
void SSD1315_ShowChar(uint8_t x, uint8_t y, char chr);
void SSD1315_ShowString(uint8_t x, uint8_t y, char *str);

#endif
