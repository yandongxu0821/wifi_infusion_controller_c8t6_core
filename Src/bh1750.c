#include "bh1750.h"
#include "cmsis_os2.h"

void BH1750_Init(void)
{
    uint8_t pwr_on = 0x01; // Power On
    uint8_t reset  = 0x07; // Reset
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &pwr_on, 1, 100);
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &reset, 1, 100);
    HAL_Delay(10);
}

void BH1750_Start(uint8_t mode)
{   
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &mode, 1, 100);
    HAL_Delay(180);
}

uint16_t BH1750_Read(void)
{
    uint8_t buf[2];
    HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDR, buf, 2, 100);
    return (uint16_t)((uint16_t)buf[0] << 8) | buf[1];
}
