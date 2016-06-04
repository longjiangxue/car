/***********************************************************************************************
/                             oled模块函数----h文件
/-----------------------------------------------------------------------------------------------
/----淘宝：驰瑞纵横电子科技			http://crzongheng.taobao.com
/----版本：V1.0		
/----时间：2013-10-18
/----说明：
***********************************************************************************************/

#ifndef __OLED_H__
#define __OLED_H__
#include "oled.h"
#include "common.h"

#define     LCD_SCL_L      gpio_set (PTE10, 0);
#define     LCD_SCL_H      gpio_set (PTE10, 1);

#define     LCD_SDA_L      gpio_set (PTE11, 0);
#define     LCD_SDA_H      gpio_set (PTE11, 1);

#define     LCD_RST_L      gpio_set (PTE12, 0);
#define     LCD_RST_H      gpio_set (PTE12, 1);

#define     LCD_DC_L       gpio_set (PTE24, 0);
#define     LCD_DC_H       gpio_set (PTE24, 1);

#define byte uint8
#define word uint16
//extern uint8  F16x16[];
//extern uint8  F6x8[][6];
//extern uint8  F8X16[];
extern uint8   BMP1[];
extern uint8   BMP2[];
#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel	    ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xCF 
#define X_WIDTH 	128
#define Y_WIDTH 	64
//***************************************函数声明***********************************************
extern void LCD_DLY_ms(uint32 ms);
//extern void LCD_WrDat(uint8 dat);
//extern void LCD_WrCmd(uint8 cmd);
extern void LCD_Set_Pos(unsigned char x, unsigned char y) ;
extern void LCD_Fill(uint8 bmp_dat);
extern void LCD_CLS(void);
extern void LCD_Init(void);
extern void LCD_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
extern void LCD_P8x16Str(uint8 x, uint8 y,uint8 ch[]);
extern void LCD_P16x16Ch(uint8 x, uint8 y,uint8 N);
extern void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 BMP[]);
 
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void LCD_P12x16Ch(uint8 x, uint8 y,uint8 N);
 void LCD_P6x8number(uint8 x,uint8 y,int16 num);
 void CLS(uint8 x,uint8 y);


#endif
 //****************************************END***************************************************
