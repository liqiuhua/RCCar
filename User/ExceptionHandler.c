

#include "ExceptionHandler.h"
#include "stm32f4xx_hal.h"
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET);
  /* USER CODE END Error_Handler_Debug */
}
