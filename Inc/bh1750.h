#ifndef __BH1750_H
#define __BH1750_H

#include "i2c.h"

#define BH1750_ADDR 0x46

void BH1750_Init(void);
void BH1750_Start(uint8_t mode);
uint16_t BH1750_Read(void);

#endif
