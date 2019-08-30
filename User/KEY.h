#ifndef __KEY_H_
#define __KEY_H_

#include "stdint.h"

#define KEY_ONE_CLICK     0x01
#define KEY_TWO_CLICK     0x02
#define KEY_THREE_CLICK   0x04
#define KEY_FOUR_CLICK    0x08
#define KEY_FIVE_CLICK    0x10
#define KEY_SIX_CLICK     0x20


#define KEY_UP_CLICK            KEY_ONE_CLICK
#define KEY_LEFT_CLICK          KEY_TWO_CLICK  
#define KEY_DOWN_CLICK         KEY_THREE_CLICK
#define KEY_RIGHT_CLICK          KEY_FOUR_CLICK
#define KEY_ENTER_CLICK         KEY_FIVE_CLICK
#define KEY_CANCEL_CLICK        KEY_SIX_CLICK

void KEY_Init(void);
uint8_t getKeyStatus(void);

#endif

