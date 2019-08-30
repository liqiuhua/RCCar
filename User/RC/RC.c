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

#define RC_MAX_VALUE 15200
#define RC_MID_VALUE 12000
#define RC_MIN_VALUE 8800

struct _RCCarCortol{
	uint16_t throttle;
	uint16_t left;
	uint16_t right;
	uint8_t travelDirection;
}RCCarCortol;

static void RCCar_Contorl(struct _RCCarCortol * carCortol);
static void RCCar_InputDeal(void);
static void RC_Init(void);
static uint32_t LCD_PWM_INFO[7];
#define PWMOUT_ONE      0
#define PWMOUT_TWO      1
#define PWMINPUT_ONE    2
#define PWMINPUT_TWO    3
#define PWMINPUT_THREE  4
#define PWMINPUT_FOUR   5
#define SERVO_RIGHT_OUT       6
#define SERVO_LEFT_OUT       7
#define CAR_FORWARD     1
#define CAR_BACKWARD    2

TaskHandle_t xHandleTaskRC=NULL;
void vRCStart(void *pvParameters)
{
    PWM_INPUT_Init();
    PWM_OUTPUT_Init();
		RC_Init();
    printf("vRCStart\n");
    while(1)
    {
        vTaskDelay(1);
       // GPIO_WriteBit(GPIOE,  GPIO_Pin_1, GPIO_Pin_SET);
      //  vTaskDelay(500);
							RCCar_InputDeal();
        RCCar_Contorl(&RCCarCortol);
    }
    
}
#include "KEY.h"
#include "TFTLCD.h"
#include "string.h"
static uint16_t PWMOUTDATA=5000;
static uint16_t ServoPWMOutData =13000;
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
		uint16_t count=0;
    while(1)
    {
        vTaskDelay(5);
				count++;
        key_value=getKeyStatus();
        if(key_value!=0)
        {
            KEYDeal(key_value);
            printf("KEY=%x \n",key_value);
            printf("get=%d\n",getTIM3ICValue(0));
					
            LCDPWM_INFO_LINE_DISPLAY(SERVO_RIGHT_OUT,RCCarCortol.right);
						LCDPWM_INFO_LINE_DISPLAY(SERVO_LEFT_OUT,RCCarCortol.left);

        }

				if(count>100)
				{
					count=0;
//					  LCDPWM_INFO_LINE_DISPLAY(PWMOUT_ONE,getTIM4PWMValue(3));
//            LCDPWM_INFO_LINE_DISPLAY(PWMOUT_TWO,getTIM4PWMValue(4));
//					  LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_ONE,getTIM3ICValue(0));
//            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_TWO,getTIM3ICValue(1));
//            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_THREE,getTIM3ICValue(2));
//            LCDPWM_INFO_LINE_DISPLAY(PWMINPUT_FOUR,getTIM3ICValue(3));

            LCDPWM_INFO_LINE_DISPLAY(SERVO_RIGHT_OUT,RCCarCortol.right);
						LCDPWM_INFO_LINE_DISPLAY(SERVO_LEFT_OUT,RCCarCortol.left);
				}
				

    }
    
}
static void KEYDeal(uint8_t KeyValue)
{
    uint8_t Display[30]={0}; 
		PWMOUTDATA=RCCarCortol.throttle;
    if(KeyValue&KEY_UP_CLICK)
    {
				RCCarCortol.travelDirection =1;
        snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_UP_CLICK",PWMOUTDATA);

    }
    if(KeyValue&KEY_DOWN_CLICK)
    {
        
        RCCarCortol.travelDirection=2;
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
        if(PWMOUTDATA<=100)
        {
            PWMOUTDATA=100;
        }
        snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_RIGHT_CLICK",PWMOUTDATA);
    }
    if(KeyValue&KEY_ENTER_CLICK)
    {
        ServoPWMOutData+=100;
       PWM_out(0x01,ServoPWMOutData);
         snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_ENTER_CLICK ",ServoPWMOutData);
    }
    if(KeyValue&KEY_CANCEL_CLICK)
    {
        ServoPWMOutData-=100;
        if(ServoPWMOutData<=100)
        {
            ServoPWMOutData=100;
        }
				PWM_out(0x01,ServoPWMOutData);
        snprintf((char *)Display,30,"%s %d",(uint8_t *)"KEY_CANCEL_CLICK",ServoPWMOutData);
    }
		RCCarCortol.throttle= PWMOUTDATA;
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
        snprintf((char *)Display,30,"Servo right= %d",value);

        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+6),0,Display);
        break;

        case (LCD_PWM_INFO_START_LINE+7):
        snprintf((char *)Display,30,"Servo left= %d",value);
        LCD_ShowStringLine((LCD_PWM_INFO_START_LINE+7),0,Display);
        break;


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


#define CAR_SERVO_MID_VALUE   13000
#define CAR_SERVO_MIN_VALUE   9000
#define CAR_SERVO_MAX_VALUE   17000
#define CAR_SERVO(value)  (value<=(CAR_SERVO_MID_VALUE-CAR_SERVO_MIN_VALUE))

static void RCCar_Contorl(struct _RCCarCortol * carCortol)
{
    if(CAR_FORWARD==RCCarCortol.travelDirection)
    {
        PWM_out(0x04,0);
        PWM_out(0x03,carCortol->throttle);
    }
    else if(CAR_BACKWARD==RCCarCortol.travelDirection)
    {
        PWM_out(0x03,0);
        PWM_out(0x04,carCortol->throttle);
    }
    else
    {
        PWM_out(0x03,0);
        PWM_out(0x04,0);
    }
		if(CAR_SERVO(carCortol->left)&&CAR_SERVO(carCortol->right))
		{
			if((carCortol->left))
			{
					PWM_out(0x01,CAR_SERVO_MID_VALUE-carCortol->left);        
			}
			else if((carCortol->right))
			{
					 PWM_out(0x01,(carCortol->right+CAR_SERVO_MID_VALUE));  
			}
			else
			{
				PWM_out(0x01,CAR_SERVO_MID_VALUE); 
			}
		}
		else
		{
			 PWM_out(0x01,CAR_SERVO_MID_VALUE);  
		}
}
static void RCCar_InputDeal(void)
{
	if(getTIM3ICValue(3)>(RC_MID_VALUE+1000))
	{
		RCCarCortol.travelDirection=CAR_BACKWARD;
		
	}
	else if(getTIM3ICValue(3)<(RC_MID_VALUE-1000))
	{
		RCCarCortol.travelDirection=CAR_FORWARD;
	}
	
	if(getTIM3ICValue(2)>(RC_MIN_VALUE+200))
	{
		RCCarCortol.throttle= (getTIM3ICValue(2)-RC_MIN_VALUE)/100*625;
	}
	else
	{
		RCCarCortol.throttle=0;
	}
	
	if(getTIM3ICValue(1)>(RC_MID_VALUE+100))
	{
		RCCarCortol.left = (getTIM3ICValue(1)-RC_MID_VALUE)*10/8;
		if(!CAR_SERVO(RCCarCortol.left))
		{
			RCCarCortol.left=4000;
		}
		RCCarCortol.right = 0;
	}
	else if(getTIM3ICValue(1)<(RC_MID_VALUE-100))
	{
		RCCarCortol.left = 0;
		RCCarCortol.right = (RC_MID_VALUE-getTIM3ICValue(1))*10/8 ;
				if(!CAR_SERVO(RCCarCortol.right))
		{
			RCCarCortol.right=4000;
		}
	}
	else
	{
		RCCarCortol.left = 0;
		RCCarCortol.right = 0;
	}
}
static void RC_Init(void)
{
	PWM_out(0x03,0);
	PWM_out(0x04,0);
	PWM_out(0x01,13000);
}
