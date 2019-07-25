
#include "FreeRTOS_Debug.h"
#include "stm32f4xx_hal.h"
#include "ExceptionHandler.h"
#include "stdio.h"
static void FreeRTOSDebugInit(void);
TIM_HandleTypeDef htim6;
TaskHandle_t xHandleTaskSystemDebug = NULL;
/* 被系统调用 */
volatile uint32_t ulHighFrequencyTimerTicks = 0UL;

void vSystemDebugStart(void *pvParameters)
{
    FreeRTOSDebugInit();
    printf("vSystemDebugStart\n");
    uint8_t pcWriteBuffer[1024];
    while(1)
    {
//        printf("=================================================\r\n");
//        printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
//        vTaskList((char *)&pcWriteBuffer);
//        printf("%s\r\n", pcWriteBuffer);

//        printf("\r\n任务名       运行计数         使用率\r\n");
//        vTaskGetRunTimeStats((char *)&pcWriteBuffer);
//        printf("%s\r\n", pcWriteBuffer);
        vTaskDelay(30000);
    }
}

static void FreeRTOSDebugInit(void)
{
    /*使用TIM6，定时50us*/ 
    
    TIM_MasterConfigTypeDef sMasterConfig={0};
    htim6.Instance = TIM6;
    htim6.Init.Prescaler=2;//时钟源是84MHz
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1600;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,15,15); 
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
      {
        Error_Handler();
      }
       HAL_TIM_Base_Start_IT(&htim6);
}
/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  //  printf("TIM6_DAC_IRQHandler\n");
  /* USER CODE END TIM6_DAC_IRQn 0 */
    ulHighFrequencyTimerTicks++;
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}
