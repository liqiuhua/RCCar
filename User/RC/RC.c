#include "RC.h"
#include "ExceptionHandler.h"
#include "PWM.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

static void KEYDeal(uint8_t KeyValue);

#define LCD_PWM_INFO_START_LINE 0

static void LCDPWM_INFODISPLAY(void);
static void LCDPWM_INFO_LINE_DISPLAY(uint8_t line, uint32_t value);
static uint32_t LCD_PWM_INFO[7];
#define PWMOUT_ONE 0
#define PWMOUT_TWO 1
#define PWMINPUT_ONE 2
#define PWMINPUT_TWO 3
#define PWMINPUT_THREE 4
#define PWMINPUT_FOUR 5
#define SERVO_OUT 6

TaskHandle_t xHandleTaskRC=NULL;
void vRCStart(void *pvParameters)
{
    PWM_INPUT_Init();
   PWM_OUTPUT_Init();
    printf("vRCStart\n");
    while(1)
    {
     //   vTaskDelay(500);
       // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET);
      //  vTaskDelay(500);
       // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_RESET);
    }
    
}
#include "KEY.h"
#include "TFTLCD.h"
#include "string.h"
void vInteractionStart(void *pvParameters)
{


    LCD.Init();
    printf("vLCDStart\n");
    uint8_t Display[30]; 
    LCDPWM_INFODISPLAY();
     KEY_Init();
    uint8_t key_value=0;
    while(1)
    {
        vTaskDelay(5);
        key_value=getKeyStatus();
        if(key_value!=0)
        {
            //KEYDeal(key_value);
            printf("KEY=%x",key_value);
            
            printf("get=%d\n",getTIM3ICValue(0));
            
            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_ONE,getTIM3ICValue(0));
            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_TWO,getTIM3ICValue(1));
        }


    }
    
}
static void KEYDeal(uint8_t KeyValue)
{
    uint8_t Display[30]={0}; 
    if(KeyValue&KEY_UP_CLICK)
    {
        snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_UP_CLICK");

    }
    if(KeyValue&KEY_DOWN_CLICK)
    {
        
         snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_DOWN_CLICK");
    }
    if(KeyValue&KEY_LEFT_CLICK)
    {
        
         snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_LEFT_CLICK");
    }
    if(KeyValue&KEY_RIGHT_CLICK)
    {
        
        snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_RIGHT_CLICK");
    }
    if(KeyValue&KEY_ENTER_CLICK)
    {
        
         snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_ENTER_CLICK");
    }
    if(KeyValue&KEY_CANCEL_CLICK)
    {
        
        snprintf((char *)Display,30,"%s",(uint8_t *)"KEY_CANCEL_CLICK");
    }
    LCD.ClearLine(1);
    LCD.ShowStringLine(1,0,Display);
    
}    

static void LCDPWM_INFO_LINE_DISPLAY(uint8_t line, uint32_t value)
{
    uint8_t Display[30]={0}; 
    switch(line)
    {
        case (LCD_PWM_INFO_START_LINE+0):
            
            snprintf((char *)Display,30,"PWM1_Out= %d",value);
            LCD.ClearLine((LCD_PWM_INFO_START_LINE+0));
            LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+0),0,Display);
        break;
        
        case (LCD_PWM_INFO_START_LINE+1):
            snprintf((char *)Display,30,"PWM2_Out= %d",value);
            LCD.ClearLine((LCD_PWM_INFO_START_LINE+1));
            LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+1),0,Display);
        break;
        case (LCD_PWM_INFO_START_LINE+2):
        snprintf((char *)Display,30,"RC1_In= %d",value);
        LCD.ClearLine((LCD_PWM_INFO_START_LINE+2));
        LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+2),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+3):
        snprintf((char *)Display,30,"RC2_In= %d",value);
        LCD.ClearLine((LCD_PWM_INFO_START_LINE+3));
        LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+3),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+4):
        snprintf((char *)Display,30,"RC3_In= %d",value);
        LCD.ClearLine((LCD_PWM_INFO_START_LINE+4));
        LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+4),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+5):
        snprintf((char *)Display,30,"RC4_In= %d",value);
        LCD.ClearLine((LCD_PWM_INFO_START_LINE+5));
        LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+5),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+6):
        snprintf((char *)Display,30,"Servo Out= %d",value);
        LCD.ClearLine((LCD_PWM_INFO_START_LINE+6));
        LCD.ShowStringLine((LCD_PWM_INFO_START_LINE+6),0,Display);
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
