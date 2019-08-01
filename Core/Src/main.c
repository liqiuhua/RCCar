/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

#include "ExceptionHandler.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

#include "FreeRTOS_Debug.h"
#include "RC.h"


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

static void BSP_Init(void);

static void AppTaskCreate (void);
static void vLEDStart(void *pvParameters);
static TaskHandle_t xHandleTaskLED = NULL;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Configure the system clock */
  SystemClock_Config();

    
/*
����������ǰ��Ϊ�˷�ֹ��ʼ�� STM32 ����ʱ���жϷ������ִ�У������ֹȫ���ж�(���� NMI �� HardFault)��
�������ĺô��ǣ�
1. ��ִֹ�е��жϷ���������� FreeRTOS �� API ������
2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
����ֲ�ļ� port.c �еĺ��� prvStartFirstTask �л����¿���ȫ���жϡ�ͨ��ָ�� cpsie i ������ __set_PRIMASK(1)
�� cpsie i �ǵ�Ч�ġ�
*/


__set_PRIMASK(1);


    BSP_Init();

//   MX_GPIO_Init();
//   GPIO_SetBits(GPIOE,GPIO_Pin_0); 
//    vRCStart(NULL);
    
  AppTaskCreate ();  
    
    /* �������ȣ� ��ʼִ������ */
vTaskStartScheduler();

/*
���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
heap �ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ� FreeRTOSConfig.h �ļ��ж���� heap ��С��
#define configTOTAL_HEAP_SIZE ( ( size_t ) ( 17 * 1024 ) )
*/
while(1){}
}

static void BSP_Init(void)
{
    MX_USART1_UART_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,ENABLE);
}
static void AppTaskCreate (void)
{
    xTaskCreate( vLEDStart,     		/* ������  */
                 "vLEDTaskStart",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 31,              		/* �������ȼ�*/
                 &xHandleTaskLED );   /* ������  */
    xTaskCreate( vSystemDebugStart,     		/* ������  */
                 "vSystemDebugStart",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 30,              		/* �������ȼ�*/
                 &xHandleTaskSystemDebug );   /* ������  */
        xTaskCreate( vRCStart,     		/* ������  */
                 "vRCStart",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 20,              		/* �������ȼ�*/
                 &xHandleTaskRC );   /* ������  */
                 
        xTaskCreate( vInteractionStart,     		/* ������  */
                 "vInteractionStart",   		/* ������    */
                 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 21,              		/* �������ȼ�*/
                 NULL );   /* ������  */
}


static void vLEDStart(void *pvParameters)
{
      /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
//  MX_USB_OTG_FS_HCD_Init();
    
   printf("vLEDStart\n"); 

  while (1)
  {
    /* USER CODE END WHILE */
		vTaskDelay(500);

			GPIO_SetBits(GPIOE,GPIO_Pin_0); 
            
		vTaskDelay(500);
			GPIO_ResetBits(GPIOE,GPIO_Pin_0);

    /* USER CODE BEGIN 3 */
  }
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	ErrorStatus HSEStartUpStatus;
    
	//SystemInit();             
	RCC_DeInit();                 //��λRCCģ��ļĴ���,��λ��ȱʡֵ
	RCC_HSEConfig(RCC_HSE_ON);  //����HSEʱ��
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); //��ȡHSE����״̬

	if(HSEStartUpStatus == SUCCESS)               //���HSE�����ɹ�
	{
//		FLASH_PrefetchBufferCmd(ENABLE);          //����FLASH��Ԥȡ����
//		FLASH_SetLatency(FLASH_Latency_2);       //FLASH�ӳ�2�����ڣ�������Ҳ�����ף����ðɣ�
		RCC_HCLKConfig(RCC_SYSCLK_Div1);         //����HCLK,PCLK2,PCLK1,PLL
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            //����ϵͳʱ��
        RCC_PLLConfig(RCC_PLLSource_HSE,25,336,2,7);
		//RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_1);
		RCC_PLLCmd(ENABLE);                                        //����PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET)                        
		{}                                                          //�ȴ�PLL�������
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);            //����ϵͳʱ��
		while(RCC_GetSYSCLKSource() !=0x08)          
		{} 
	}
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
}
int fputc(int ch,FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);
	
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	
	
	return (ch);
}
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  USART_InitTypeDef USART_InitStruct;
  	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
    
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);  //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);          //��ʼ��PA9��PA10
	
	


  USART_DeInit(USART1);
  /* �ж�ʽ��˫��/ȫ˫��ͨ��, ��������ò�Ҫ����1Mbps, ����ͨ�Ž���ʹ��DMA��ʽ */
  USART_InitStruct.USART_BaudRate = 115200; /* ʵ�鲨����, 3.5Mbps */
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  
  USART_Init(USART1,&USART_InitStruct);
  


  /* Configure USART1 USART_IT_RXNE */
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
//USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
//  USART_OverSampling8Cmd(USART1,ENABLE);
//  USART_OverSampling8Cmd(USART6,ENABLE);
  
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE); 

  USART_ClearFlag(USART1, USART_FLAG_TC);
	
	
	
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{


}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


  /*Configure GPIO pins : LCD_CS_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStruct);



}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
