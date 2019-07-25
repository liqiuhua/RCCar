
#include "TFTLCD_App.h"
#include "stdio.h"
TaskHandle_t xHandleTaskLCD = NULL;

void vLCDStart(void *pvParameters)
{

    while(1)
    {
        vTaskDelay(500);
    }
}
