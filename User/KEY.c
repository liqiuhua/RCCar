#include "KEY.h"
#include "ExceptionHandler.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

#define KEY_ONE_PIN             GPIO_PIN_2
#define KEY_TWO_PIN             GPIO_PIN_3
#define KEY_THREE_PIN           GPIO_PIN_4
#define KEY_FOUR_PIN            GPIO_PIN_5
#define KEY_FIVE_PIN            GPIO_PIN_6
#define KEY_ONE_FIVE_PORT       GPIOE

#define KEY_SIX_PIN             GPIO_PIN_0
#define KEY_SIX_PORT            GPIOA

static uint8_t KEY_Value = 0x00;
void KEY_Init(void)
{
     GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();


  /*Configure GPIO pin */
  GPIO_InitStruct.Pin = KEY_ONE_PIN |KEY_TWO_PIN|KEY_THREE_PIN|KEY_FOUR_PIN|KEY_FIVE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_ONE_FIVE_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = KEY_SIX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_SIX_PORT, &GPIO_InitStruct);
}
uint8_t getKeyStatus(void)
{

    if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_ONE_PIN)==RESET)
    {
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_ONE_PIN)==RESET)
        {
            KEY_Value |=KEY_ONE_CLICK;
        }
    }
    else
    {
        KEY_Value &= (~KEY_ONE_CLICK);
    }
    
    if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_TWO_PIN)==RESET)
    {
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_TWO_PIN)==RESET)
        {
            KEY_Value |=KEY_TWO_CLICK;
        }
    }
    else
    {
        KEY_Value &= (~KEY_TWO_CLICK);
    }
        if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_THREE_PIN)==RESET)
    {
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_THREE_PIN)==RESET)
        {
            KEY_Value |=KEY_THREE_CLICK;
        }
    }
    else
    {
        KEY_Value &= (~KEY_THREE_CLICK);
    }
    
    if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_FOUR_PIN)==RESET)
    {
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_FOUR_PIN)==RESET)
        {
            KEY_Value |=KEY_FOUR_CLICK;
        }
    }
    else
    {
        KEY_Value &= (~KEY_FOUR_CLICK);
    }
    
    if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_FIVE_PIN)==RESET)
    {
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(KEY_ONE_FIVE_PORT, KEY_FIVE_PIN)==RESET)
        {
            KEY_Value |=KEY_FIVE_CLICK;
        }
    }
    else
    {
        KEY_Value &= (~KEY_FIVE_CLICK);
    }
        if(HAL_GPIO_ReadPin(KEY_SIX_PORT, KEY_SIX_PIN)==RESET)
    {
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(KEY_SIX_PORT, KEY_SIX_PIN)==RESET)
        {
            KEY_Value |=KEY_SIX_CLICK;
        }
    }
    else
    {
        KEY_Value &= (~KEY_SIX_CLICK);
    }
    
    return KEY_Value;
}
