#include "PWM.h"
#include "stm32f4xx_hal.h"
#include "ExceptionHandler.h"
#include "stdio.h"
TIM_HandleTypeDef TIM3_Handler;


uint32_t  TIM3_IC_Value[4];
void PWM_INPUT_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PC6     ------> TIM3_CH1
    PC7     ------> TIM3_CH2
    PC8     ------> TIM3_CH3
    PC9     ------> TIM3_CH4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    
    
    TIM3_Handler.Instance = TIM3;
    TIM3_Handler.Init.Prescaler = 83;
    TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM3_Handler.Init.Period = 0xFFFFFFFF;
    TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM3_Handler);
        /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 13,13);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
     TIM_IC_InitTypeDef TIM3_sConfigIC = {0};
    TIM3_sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    TIM3_sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM3_sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&TIM3_Handler, &TIM3_sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&TIM3_Handler, &TIM3_sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
//    if (HAL_TIM_IC_ConfigChannel(&TIM3_Handler, &TIM3_sConfigIC, TIM_CHANNEL_3) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    if (HAL_TIM_IC_ConfigChannel(&TIM3_Handler, &TIM3_sConfigIC, TIM_CHANNEL_4) != HAL_OK)
//    {
//        Error_Handler();
//    }
    //__HAL_TIM_ENABLE_IT(&TIM3_Handler,TIM_IT_UPDATE);
   // __HAL_TIM_ENABLE(&TIM3_Handler);
    HAL_TIM_IC_Start_IT(&TIM3_Handler,TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&TIM3_Handler,TIM_CHANNEL_2);
 //   HAL_TIM_IC_Start_IT(&TIM3_Handler,TIM_CHANNEL_3);
}
TIM_HandleTypeDef TIM4_Handler;
void PWM_OUTPUT_Init(void)
{
      GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    PD14     ------> TIM4_CH3
    PD15     ------> TIM4_CH4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
      
     
    TIM4_Handler.Instance = TIM4;
    TIM4_Handler.Init.Prescaler = 7;
    TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM4_Handler.Init.Period = 36000;
    TIM4_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM4_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_OC_Init(&TIM4_Handler);

    
     TIM_OC_InitTypeDef TIM4_sConfigOC = {0};
     
    TIM4_sConfigOC.OCMode = TIM_OCMODE_PWM1;
    TIM4_sConfigOC.Pulse = 5000;
    TIM4_sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM4_sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&TIM4_Handler, &TIM4_sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_OC_ConfigChannel(&TIM4_Handler, &TIM4_sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_OC_ConfigChannel(&TIM4_Handler, &TIM4_sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_OC_ConfigChannel(&TIM4_Handler, &TIM4_sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    
   // __HAL_TIM_ENABLE(&TIM4_Handler);
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_4);

}
void PWM_out(uint8_t ch,uint16_t data)
{
    if(data>36000)
        data=36000;
    switch(ch)
    {
        case 1:
            TIM4->CCR1=data; 
        break;
        case 2:
            TIM4->CCR2=data; 
        break;
        case 3:
            TIM4->CCR3=data; 
        break;
        case 4:
            TIM4->CCR4=data; 
        break;
        case 0xff:
         TIM4->CCR1=data; 
        TIM4->CCR2=data; 
        TIM4->CCR3=data; 
        TIM4->CCR4=data; 
            break;
        default:
        break;
    }
}
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}

void TIM3_IC_Callback(TIM_HandleTypeDef *htim)
{
    static uint32_t ICValuePre[4],ICValueLast[4];
    uint16_t SR=0;
//    if(htim->Instance ==TIM3)
//    {
//    ICValueLast[0] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
//    printf("TIM3_IC_Callback %d \n",ICValueLast[0]-ICValuePre[0]);
//    ICValuePre[0] = ICValueLast[0];
//    TIM3_IC_Value .ChannelOneValue = ICValueLast[0]-ICValuePre[0];
//    }
    printf("TIM3_IC_Callback SR = %x \n",TIM3->SR);
    SR=TIM3->SR;
   // if(SR&TIM_FLAG_CC1)
    {
        ICValueLast[0] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
    //    printf("TIM3_IC_Callback 1 %d \n",ICValueLast[0]-ICValuePre[0]);
        
        TIM3_IC_Value[0] = ICValueLast[0]-ICValuePre[0];
        
        ICValuePre[0] = ICValueLast[0];
        TIM3->SR &=(~TIM_FLAG_CC1) ;
        //__HAL_TIM_CLEAR_FLAG(&TIM3_Handler, TIM_FLAG_CC1|TIM_FLAG_CC1OF);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_RESET);
      //  printf("TIM3_IC_Callback 1 \n");
    }
        if(SR&TIM_FLAG_CC2)
    {
        ICValueLast[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
      //  printf("TIM3_IC_Callback 2 %d \n",ICValueLast[1]-ICValuePre[1]);
        
        TIM3_IC_Value[1] = ICValueLast[1]-ICValuePre[1];
        ICValuePre[1] = ICValueLast[1];
        __HAL_TIM_CLEAR_FLAG(&TIM3_Handler, TIM_FLAG_CC2|TIM_FLAG_CC2OF);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET);
       // printf("TIM3_IC_Callback 2 \n");
    }
    
      //    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET);
        
    
}
uint32_t getTIM3ICValue(uint8_t channel)
{
    if(channel>3)
        return 0;
        
    return TIM3_IC_Value[channel];
}


