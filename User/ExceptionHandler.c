

#include "ExceptionHandler.h"
#include "stm32f4xx.h"
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
   // GPIO_WriteBit(GPIOE,  GPIO_Pin_1, GPIO_Pin_SET);
  /* USER CODE END Error_Handler_Debug */
}
