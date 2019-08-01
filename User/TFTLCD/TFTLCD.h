#ifndef __TFTLCD_H_
#define __TFTLCD_H_

#include "stdint.h"
typedef struct
{
    uint16_t id;
    uint16_t width;
    uint16_t height;
    uint8_t dir;
    uint16_t WramCmd;
    uint16_t SetXPosCmd;
    uint16_t SetYPosCmd;
    void (*Init)(void);
    void (*BlackLight)(uint8_t status);
    void (*Clear)(uint16_t Color);
    void (*DisplayDir)(uint8_t status);
    void (*ShowString)(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p);
    void (*ShowStringLine)(uint16_t row,uint16_t column,uint8_t *p);
     void (*ClearLine)(uint16_t row);
}_TFTLCD;


extern _TFTLCD LCD;

void LCD_WRITE_DATA(uint16_t data);
void LCD_WRITE_REG(uint16_t data);
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y);
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
#endif
