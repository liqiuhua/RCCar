#ifndef __RC_H_
#define __RC_H_


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"


extern TaskHandle_t xHandleTaskRC;

void vRCStart(void *pvParameters);   
void vInteractionStart(void *pvParameters);
#endif

