#include "PWM.h"
#include "stm32f4xx.h"
#include "ExceptionHandler.h"
#include "stdio.h"



uint32_t  TIM3_IC_Value[4];
void PWM_INPUT_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;			//����TME3��Ϣ�ṹ��;
  TIM_ICInitTypeDef  TIM3_ICInitStructure;
    /* Peripheral clock enable */
  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /**TIM3 GPIO Configuration    
    PC6     ------> TIM3_CH1
    PC7     ------> TIM3_CH2
    PC8     ------> TIM3_CH3
    PC9     ------> TIM3_CH4 
    */
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);  
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3); 
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);  
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3); 
  
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFFFFFF; 	     //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=20;      //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	//TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	
    
    
        /* TIM3 interrupt Init */
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x06; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
	//��ʼ��TIM3���벶�����
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//�����ز���
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	
	
    
    TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��5
}
void PWM_OUTPUT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
//    /* Peripheral clock enable */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//    /**TIM4 GPIO Configuration    
//    PD12     ------> TIM4_CH1
//    PD13     ------> TIM4_CH2
//    PD14     ------> TIM4_CH3
//    PD15     ------> TIM4_CH4 
//    */
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);  
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);  
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); 
      
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    TIM_TimeBaseStructure.TIM_Prescaler=83;//13;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period =40000;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//��ʼ����ʱ��

	//��ʼ��TIM4 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_Pulse = 5000;
    
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4OC1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���

    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4OC2
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
 	
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4OC3
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR3�ϵ�Ԥװ�ؼĴ���
 	
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4OC4
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR4�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
   

}
void PWM_out(uint8_t ch,uint16_t data)
{
    if(data>40000)
        data=40000;
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
    TIM_SetAutoreload(TIM4,40000);
}

uint8_t TIM3CH1_CAPTURE_STA=0;
void TIM3_IRQHandler(void)
{
   
    static uint32_t ICValuePre[4],ICValueLast[4];
    if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {   
        if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_6)==SET)
        {
            ICValuePre[0] = TIM_GetCapture1(TIM3);    
        }
        else if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_6)==RESET)
        {
            ICValueLast[0] = TIM_GetCapture1(TIM3);
            if(ICValuePre[0]>ICValueLast[0])       
            {
                TIM3_IC_Value[0] =65535-ICValuePre[0]+ICValueLast[0];
            }
            else
            {                   
                TIM3_IC_Value[0] = ICValueLast[0]-ICValuePre[0];
            }
        }  
        TIM3->SR &=(~TIM_FLAG_CC1) ;

    }
    if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
    {   
        if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_7)==SET)
        {
            ICValuePre[1] = TIM_GetCapture2(TIM3);    
        }
        else if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_7)==RESET)
        {
            ICValueLast[1] = TIM_GetCapture2(TIM3);
            if(ICValuePre[1]>ICValueLast[1])       
            {
                TIM3_IC_Value[1] =65535-ICValuePre[1]+ICValueLast[1];
            }
            else
            {                   
                TIM3_IC_Value[1] = ICValueLast[1]-ICValuePre[1];
            }
        }  
        TIM3->SR &=(~TIM_FLAG_CC2) ;

    }
    if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
    {   
        if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_8)==SET)
        {
            ICValuePre[2] = TIM_GetCapture3(TIM3);    
        }
        else if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_8)==RESET)
        {
            ICValueLast[2] = TIM_GetCapture3(TIM3);
            if(ICValuePre[2]>ICValueLast[2])       
            {
                TIM3_IC_Value[2] =65535-ICValuePre[2]+ICValueLast[2];
            }
            else
            {                   
                TIM3_IC_Value[2] = ICValueLast[2]-ICValuePre[2];
            }
        }  
        TIM3->SR &=(~TIM_FLAG_CC3) ;

    }
        if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
    {   
        if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_9)==SET)
        {
            ICValuePre[3] = TIM_GetCapture4(TIM3);    
        }
        else if(GPIO_ReadInputDataBit(GPIOC,  GPIO_Pin_9)==RESET)
        {
            ICValueLast[3] = TIM_GetCapture4(TIM3);
            if(ICValuePre[3]>ICValueLast[3])       
            {
                TIM3_IC_Value[3] =65535-ICValuePre[3]+ICValueLast[3];
            }
            else
            {                   
                TIM3_IC_Value[3] = ICValueLast[3]-ICValuePre[3];
            }
        }  
        TIM3->SR &=(~TIM_FLAG_CC4) ;

    }
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}

uint32_t getTIM3ICValue(uint8_t channel)
{
    if(channel>3)
        return 0;
        
    return TIM3_IC_Value[channel];
}
uint32_t getTIM4PWMValue(uint8_t channel)
{
    if(channel>4)
        return 0;
    switch(channel)
    {
        case 1:
            return TIM4->CCR1;
        case 2:
            return TIM4->CCR2;
        case 3:
            return TIM4->CCR3;
        case 4:
            return TIM4->CCR4;
        default:
            break;
        
    }
}

