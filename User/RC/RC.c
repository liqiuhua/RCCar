#include "RC.h"
#include "ExceptionHandler.h"
#include "PWM.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "GUI.h"

static void KEYDeal(uint8_t KeyValue);

#define LCD_PWM_INFO_START_LINE 0

static void LCDPWM_INFODISPLAY(void);
static void LCDPWM_INFO_LINE_DISPLAY(uint8_t line, uint32_t value);
static void LCD_ShowStringLine(uint16_t row,uint16_t column,uint8_t *p);
static uint32_t LCD_PWM_INFO[7];
#define PWMOUT_ONE      0
#define PWMOUT_TWO      1
#define PWMINPUT_ONE    2
#define PWMINPUT_TWO    3
#define PWMINPUT_THREE  4
#define PWMINPUT_FOUR   5
#define SERVO_OUT       6

TaskHandle_t xHandleTaskRC=NULL;
void vRCStart(void *pvParameters)
{
    PWM_INPUT_Init();
    PWM_OUTPUT_Init();
    printf("vRCStart\n");
    while(1)
    {
       // vTaskDelay(500);
       // GPIO_WriteBit(GPIOE,  GPIO_Pin_1, GPIO_Pin_SET);
      //  vTaskDelay(500);
        GPIO_WriteBit(GPIOE,  GPIO_Pin_1, Bit_RESET);
    }
    
}
#include "KEY.h"
#include "TFTLCD.h"
#include "string.h"
static uint16_t PWMOUTDATA=5000;
static uint8_t UP_DOWN_FLAG=1;
void vInteractionStart(void *pvParameters)
{

    
    LCD.Init();
    printf("vLCDStart\n");
    //LCDPWM_INFODISPLAY();
    
    GUI_Init();

    GUI_SetBkColor(GUI_RED);
    GUI_Clear();
    //GUI_SetColor(GUI_BLUE);
    GUI_SetFont(&GUI_Font16_ASCII);
    GUI_DispStringAt("hello word !",5,5);
    KEY_Init();
    uint8_t key_value=0;
    while(1)
    {
        vTaskDelay(5);
        key_value=getKeyStatus();
        if(key_value!=0)
        {
            KEYDeal(key_value);
            printf("KEY=%x \n",key_value);
            printf("get=%d\n",getTIM3ICValue(0));
            
            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_ONE,getTIM3ICValue(0));
            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_TWO,getTIM3ICValue(1));
            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_THREE,getTIM3ICValue(2));
            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_FOUR,getTIM3ICValue(3));
            LCDPWM_INFO_LINE_DISPLAY(PWMOUT_ONE,getTIM4PWMValue(3));
            LCDPWM_INFO_LINE_DISPLAY(PWMOUT_TWO,getTIM4PWMValue(4));
        }
        if(UP_DOWN_FLAG)
        {

            PWM_out(0x04,0);
            PWM_out(0x03,PWMOUTDATA);
        }
        else
        {
            
           PWM_out(0x03,0);
           PWM_out(0x04,(PWMOUTDATA)); 
        }

    }
    
}
static void KEYDeal(uint8_t KeyValue)
{
    uint8_t Display[30]={0}; 
    if(KeyValue&KEY_UP_CLICK)
    {

        UP_DOWN_FLAG=1;
        snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_UP_CLICK",PWMOUTDATA);

    }
    if(KeyValue&KEY_DOWN_CLICK)
    {
        
        UP_DOWN_FLAG=0;
         snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_DOWN_CLICK",PWMOUTDATA);
    }
    if(KeyValue&KEY_LEFT_CLICK)
    {
        PWMOUTDATA+=500;
         snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_LEFT_CLICK",PWMOUTDATA);
    }
    if(KeyValue&KEY_RIGHT_CLICK)
    {
        PWMOUTDATA-=500;
        snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_RIGHT_CLICK",PWMOUTDATA);
    }
    if(KeyValue&KEY_ENTER_CLICK)
    {
        
         snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_ENTER_CLICK");
    }
    if(KeyValue&KEY_CANCEL_CLICK)
    {
        RCC_ClocksTypeDef get_rcc_clock; 
        RCC_GetClocksFreq(&get_rcc_clock); 
        printf("freq:  SYSCLK = %d ,HCLK = %d ,PCLK1 = %d ,PCLK2 = %d \n",get_rcc_clock.SYSCLK_Frequency,get_rcc_clock.HCLK_Frequency,get_rcc_clock.PCLK1_Frequency,get_rcc_clock.PCLK2_Frequency);
       
        snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_CANCEL_CLICK");
    }

    LCD_ShowStringLine(9,0,Display);
    
}    

static void LCDPWM_INFO_LINE_DISPLAY(uint8_t line, uint32_t value)
{
    uint8_t Display[30]={0}; 
    switch(line)
    {
        case (LCD_PWM_INFO_START_LINE+0):
            
            snprintf((char *)Display,30,"PWM1_Out= %d",value);
            LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+0),0,Display);
        break;
        
        case (LCD_PWM_INFO_START_LINE+1):
            snprintf((char *)Display,30,"PWM2_Out= %d",value);

            LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+1),0,Display);
        break;
        case (LCD_PWM_INFO_START_LINE+2):
        snprintf((char *)Display,30,"RC1_In= %d",value);

        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+2),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+3):
        snprintf((char *)Display,30,"RC2_In= %d",value);

        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+3),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+4):
        snprintf((char *)Display,30,"RC3_In= %d",value);

        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+4),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+5):
        snprintf((char *)Display,30,"RC4_In= %d",value);
        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+5),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+6):
        snprintf((char *)Display,30,"Servo Out= %d",value);
       // LCD.ClearLine((LCD_PWM_INFO_START_LINE+6));
        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+6),0,Display);
        break;

//        case (LCD_PWM_INFO_START_LINE+7):
//        snprintf((char *)Display,30,"RC1_In= %d",value);
//        LCD.ClearLine((LCD_PWM_INFO_START_LINE+7));
//        LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+7),0,Display);
//        break;


    }
    
} 
static void LCDPWM_INFODISPLAY(void)
{
    for(uint8_t i=0;i<7;++i)
    {
       LCDPWM_INFO_LINE_DISPLAY(i, LCD_PWM_INFO[i]);
    }
}
static void LCD_ShowStringLine(uint16_t row,uint16_t column,uint8_t *p)
{        
        GUI_DispStringAt("                                                       ",5+8*column,5+16*row);
        GUI_DispStringAt((const char *)p,5+8*column,5+16*row);
}