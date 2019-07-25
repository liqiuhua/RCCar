#ifndef _TFTLCD_APP_H_
#define _TFTLCD_APP_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

void vLCDStart(void *pvParameters);
extern TaskHandle_t xHandleTaskLCD;



#endif
