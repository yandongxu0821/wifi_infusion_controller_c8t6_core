#include "ssd1315.h"
#include "font16x8.h"
#include "i2c.h"
#include "string.h"

extern I2C_HandleTypeDef hi2c1;

// 显存缓冲区：1024字节数据 + 1字节控制位(0x40)
// 使用 static 确保局部可见，使用 uint8_t 对应 I2C 传输位宽
static uint8_t OLED_GRAM[1025];

/**
 * @brief 向OLED发送命令
 * @param cmd: 要发送的命令字节
 */
void OLED_WriteCmd(uint8_t cmd)
{
  uint8_t data[2] = {0x00, cmd}; // Co=0, D/C#=0 (后面跟的是命令)
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, 2, 100);
}

/**
 * @brief 清空显存缓冲区（不立刻刷新屏幕）
 */
void SSD1315_Clear(void)
{
  // 保持第一个字节(0x40)不变，清空后续1024字节
  memset(&OLED_GRAM[1], 0, 1024);
}

/**
 * @brief 将显存数据通过DMA推送到OLED
 */
void SSD1315_Update(void)
{
  /* 1. 发送命令告诉 OLED：接下来从第0列、第0页开始接收数据 */
  OLED_WriteCmd(0x21); // Set Column Address
  OLED_WriteCmd(0x00); // Start 0
  OLED_WriteCmd(0x7F); // End 127
    
  OLED_WriteCmd(0x22); // Set Page Address
  OLED_WriteCmd(0x00); // Start 0
  OLED_WriteCmd(0x07); // End 7

  /* 2. 启动 DMA 传输显存（非阻塞） */
  if (HAL_I2C_Master_Transmit_DMA(&hi2c1, OLED_ADDR, OLED_GRAM, 1025) != HAL_OK) {
    /* DMA 启动失败，回退为阻塞传输并释放互斥 */
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, OLED_GRAM, 1025, 500);
    return;
  }
}

/**
 * @brief 初始化SSD1315
 * 配置为水平寻址模式，方便DMA一次性刷新全屏
 */
void SSD1315_Init(void)
{
  /* 基础配置 */
  OLED_WriteCmd(0xAE); // 关闭显示
  OLED_WriteCmd(0xD5); // 设置时钟分频
  OLED_WriteCmd(0x80);
  OLED_WriteCmd(0xA8); // 设置复用率
  OLED_WriteCmd(0x3F);
  OLED_WriteCmd(0xD3); // 设置显示偏移
  OLED_WriteCmd(0x00);
  OLED_WriteCmd(0x40); // 设置起始行
  
  /* 关键：设置为水平寻址模式 (Horizontal Addressing Mode) */
  OLED_WriteCmd(0x20); 
  OLED_WriteCmd(0x00); 
  
  /* 设置列地址范围 (0-127) */
  OLED_WriteCmd(0x21);
  OLED_WriteCmd(0x00);
  OLED_WriteCmd(0x7F);
  
  /* 设置页地址范围 (0-7) */
  OLED_WriteCmd(0x22);
  OLED_WriteCmd(0x00);
  OLED_WriteCmd(0x07);

  OLED_WriteCmd(0xA1); // 左右反转设置 (根据硬件接线调整)
  OLED_WriteCmd(0xC8); // 上下反转设置
  OLED_WriteCmd(0xDA); // COM硬件引脚配置
  OLED_WriteCmd(0x12);
  OLED_WriteCmd(0x81); // 对比度
  OLED_WriteCmd(0xCF);
  OLED_WriteCmd(0xD9); // 预充电周期
  OLED_WriteCmd(0xF1);
  OLED_WriteCmd(0xDB); // VCOMH电压
  OLED_WriteCmd(0x40);
  OLED_WriteCmd(0xA4); // 全屏显示开启 (跟随RAM)
  OLED_WriteCmd(0xA6); // 正常显示 (不反相)
  OLED_WriteCmd(0x8D); // 电荷泵
  OLED_WriteCmd(0x14);
  OLED_WriteCmd(0xAF); // 开启显示

  HAL_Delay(10);       // 等待屏幕稳定

  // 初始化显存控制字节
  OLED_GRAM[0] = 0x40; // 开启数据流模式
  SSD1315_Clear();
  SSD1315_Update();
}
/**
 * @brief 画点函数
 * @param x: 起始列 (0-127)
 * @param y: 起始页 (0-63)
 * @param color: 1点亮, 0熄灭
 */
void SSD1315_DrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
  if(x > 127 || y > 63) return;
  
  if(color)
    OLED_GRAM[1 + x + (y / 8) * 128] |= (1 << (y % 8));
  else
    OLED_GRAM[1 + x + (y / 8) * 128] &= ~(1 << (y % 8));
}

/**
 * @brief 显示单个字符 (16x8)
 * @param x: 起始列 (0-127)
 * @param y: 起始页 (0-7)
 * @param chr: ASCII字符
 */
void SSD1315_ShowChar(uint8_t x, uint8_t y, char chr)
{
  uint8_t i, c = chr - ' '; 
  if(x > 120 || y > 7) return;

  for(i = 0; i < 8; i++)
  {
    // 写入上半部分 (Page y)
    OLED_GRAM[1 + x + i + (y * 128)] = Font16x8[c][i];
    // 写入下半部分 (Page y+1)
    OLED_GRAM[1 + x + i + ((y + 1) * 128)] = Font16x8[c][i + 8];
  }
}

/**
 * @brief 显示字符串
 * @param x: 起始列 (0-127)
 * @param y: 起始页 (0-7)
 * @param str: null结尾字符串
 */
void SSD1315_ShowString(uint8_t x, uint8_t y, char *str)
{
  while (*str)
  {
    SSD1315_ShowChar(x, y, *str++);
    x += 8;
    if (x > 120) { x = 0; y += 2; }
    if (y > 7) break;
  }
}
