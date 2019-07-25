#ifndef __FREERTOS_DEBUG_H_
#define __FREERTOS_DEBUG_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"


extern TaskHandle_t xHandleTaskSystemDebug;

void vSystemDebugStart(void *pvParameters);   

#endif
