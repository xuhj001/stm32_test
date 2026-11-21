#ifndef __OLED_I2C_H
#define	__OLED_I2C_H


#define OLED_ADDRESS	0x78 //屏可以0x78和0x7A两个地址 -- 默认0x78

// OLED_ShowStr 的 TextSize：与 oled.c 中 case 一致
#define OLED_FONT_6X8   1   // 6x8 小字体（当前驱动未实现）
#define OLED_FONT_8X16  2   // 8x16 大字体（当前实际使用）

void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
void OLED_Test(void);
#endif
