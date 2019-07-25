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
#endif
