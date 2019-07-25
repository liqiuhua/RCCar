#include "TFTLCD.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "FONT.H"


#define LCD_RD_Pin GPIO_PIN_8
#define LCD_RD_GPIO_Port GPIOE
#define LCD_WR_Pin GPIO_PIN_9
#define LCD_WR_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LCD_BL_Pin GPIO_PIN_10
#define LCD_BL_GPIO_Port GPIOD


#define LCD_RD_SET() HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin,GPIO_PIN_SET)
#define LCD_WR_SET() HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin,GPIO_PIN_SET)
#define LCD_RS_SET() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin,GPIO_PIN_SET)
#define LCD_CS_SET() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,GPIO_PIN_SET)

#define LCD_RD_RESET() HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin,GPIO_PIN_RESET)
#define LCD_WR_RESET() HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin,GPIO_PIN_RESET)
#define LCD_RS_RESET() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin,GPIO_PIN_RESET)
#define LCD_CS_RESET() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,GPIO_PIN_RESET)

#define LCD_WRITE_16BIT(x)  GPIOB->ODR=(x)
#define LCD_READ_16BIT() GPIOB->IDR



/********************************************************************************/
/****************************LCD��ʾ����غ궨��*********************************/
/********************************************************************************/

#define LCD_WIDTH 240
#define LCD_HEIGHT 320

//LCD�Ļ�����ɫ�ͱ���ɫ	   
uint16_t POINT_COLOR=0x0000;	//������ɫ
uint16_t BACK_COLOR=0xFFFF;  //����ɫ 


//ɨ�跽����
#define L2R_U2D  0 //������,���ϵ���
#define L2R_D2U  1 //������,���µ���
#define R2L_U2D  2 //���ҵ���,���ϵ���
#define R2L_D2U  3 //���ҵ���,���µ���

#define U2D_L2R  4 //���ϵ���,������
#define U2D_R2L  5 //���ϵ���,���ҵ���
#define D2U_L2R  6 //���µ���,������
#define D2U_R2L  7 //���µ���,���ҵ���	 

#define DFT_SCAN_DIR  R2L_D2U//L2R_U2D  //Ĭ�ϵ�ɨ�跽��

//������ɫ
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 


#define SET_X_POS_CMD 0x002A
#define SET_Y_POS_CMD 0x002B
#define WRAM_CMD      0x002C

/********************************************************************************/
/****************************LCD��ʾ����غ궨�����****************************/
/********************************************************************************/


static void LCD_Init(void);
static void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue);
static uint16_t LCD_ReadReg(uint16_t LCD_Reg);
static uint16_t LCD_RD_DATA(void);
static void LCD_WRITE_DATA(uint16_t data);
static void LCD_WRITE_REG(uint16_t data);
static void LCD_BlackLight(uint8_t status);
static void LCD_Clear(uint16_t color);
static void LCD_DisplayDir(uint8_t dir);
static void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p);
static void LCD_ShowStringLine(uint16_t row,uint16_t column,uint8_t *p);
static void LCD_ClearLine(uint16_t row);
_TFTLCD LCD={
    0,
    LCD_WIDTH,
    LCD_HEIGHT,
    0,
    0,
    0,
    0,
    LCD_Init,
    LCD_BlackLight,
    LCD_Clear,
    LCD_DisplayDir,
    LCD_ShowString,
    LCD_ShowStringLine,
    LCD_ClearLine,
};
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
void delay_ms(uint16_t ms)
{
//      uint32_t Time = HAL_GetTick();

//		while((HAL_GetTick()-Time)<ms){}
    vTaskDelay(ms);
}

static void LCD_HardwareInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
    /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin
                           |LCD_CS_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);  
  
    /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static void LCD_Init(void)
{
    LCD_HardwareInit();
    delay_ms(50);
    LCD_WriteReg(0x0000,0x0001);
    delay_ms(50);
    LCD.id = LCD_ReadReg(0x0000);
    
    	LCD_WRITE_REG(0XD3);				   
		LCD_RD_DATA(); 				//dummy read 	
 		LCD_RD_DATA();   	    	//����0X00
  		LCD.id=LCD_RD_DATA();   	//��ȡ93								   
 		LCD.id<<=8;
		LCD.id|=LCD_RD_DATA();  	//��ȡ41 
    printf("LCD ID : %2x \n",LCD.id);
    if(LCD.id==0x9341)
    {
        LCD_WRITE_REG(0xCF);  
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0xC1); 
		LCD_WRITE_DATA(0X30); 
		LCD_WRITE_REG(0xED);  
		LCD_WRITE_DATA(0x64); 
		LCD_WRITE_DATA(0x03); 
		LCD_WRITE_DATA(0X12); 
		LCD_WRITE_DATA(0X81); 
		LCD_WRITE_REG(0xE8);  
		LCD_WRITE_DATA(0x85); 
		LCD_WRITE_DATA(0x10); 
		LCD_WRITE_DATA(0x7A); 
		LCD_WRITE_REG(0xCB);  
		LCD_WRITE_DATA(0x39); 
		LCD_WRITE_DATA(0x2C); 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0x34); 
		LCD_WRITE_DATA(0x02); 
		LCD_WRITE_REG(0xF7);  
		LCD_WRITE_DATA(0x20); 
		LCD_WRITE_REG(0xEA);  
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_REG(0xC0);    //Power control 
		LCD_WRITE_DATA(0x1B);   //VRH[5:0] 
		LCD_WRITE_REG(0xC1);    //Power control 
		LCD_WRITE_DATA(0x01);   //SAP[2:0];BT[3:0] 
		LCD_WRITE_REG(0xC5);    //VCM control 
		LCD_WRITE_DATA(0x30); 	 //3F
		LCD_WRITE_DATA(0x30); 	 //3C
		LCD_WRITE_REG(0xC7);    //VCM control2 
		LCD_WRITE_DATA(0XB7); 
		LCD_WRITE_REG(0x36);    // Memory Access Control 
		LCD_WRITE_DATA(0x48); 
		LCD_WRITE_REG(0x3A);   
		LCD_WRITE_DATA(0x55); 
		LCD_WRITE_REG(0xB1);   
		LCD_WRITE_DATA(0x00);   
		LCD_WRITE_DATA(0x1A); 
		LCD_WRITE_REG(0xB6);    // Display Function Control 
		LCD_WRITE_DATA(0x0A); 
		LCD_WRITE_DATA(0xA2); 
		LCD_WRITE_REG(0xF2);    // 3Gamma Function Disable 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_REG(0x26);    //Gamma curve selected 
		LCD_WRITE_DATA(0x01); 
		LCD_WRITE_REG(0xE0);    //Set Gamma 
		LCD_WRITE_DATA(0x0F); 
		LCD_WRITE_DATA(0x2A); 
		LCD_WRITE_DATA(0x28); 
		LCD_WRITE_DATA(0x08); 
		LCD_WRITE_DATA(0x0E); 
		LCD_WRITE_DATA(0x08); 
		LCD_WRITE_DATA(0x54); 
		LCD_WRITE_DATA(0XA9); 
		LCD_WRITE_DATA(0x43); 
		LCD_WRITE_DATA(0x0A); 
		LCD_WRITE_DATA(0x0F); 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0x00); 		 
		LCD_WRITE_REG(0XE1);    //Set Gamma 
		LCD_WRITE_DATA(0x00); 
		LCD_WRITE_DATA(0x15); 
		LCD_WRITE_DATA(0x17); 
		LCD_WRITE_DATA(0x07); 
		LCD_WRITE_DATA(0x11); 
		LCD_WRITE_DATA(0x06); 
		LCD_WRITE_DATA(0x2B); 
		LCD_WRITE_DATA(0x56); 
		LCD_WRITE_DATA(0x3C); 
		LCD_WRITE_DATA(0x05); 
		LCD_WRITE_DATA(0x10); 
		LCD_WRITE_DATA(0x0F); 
		LCD_WRITE_DATA(0x3F); 
		LCD_WRITE_DATA(0x3F); 
		LCD_WRITE_DATA(0x0F); 
		LCD_WRITE_REG(0x2B); 
		LCD_WRITE_DATA(0x00);
		LCD_WRITE_DATA(0x00);
		LCD_WRITE_DATA(0x01);
		LCD_WRITE_DATA(0x3f);
		LCD_WRITE_REG(0x2A); 
		LCD_WRITE_DATA(0x00);
		LCD_WRITE_DATA(0x00);
		LCD_WRITE_DATA(0x00);
		LCD_WRITE_DATA(0xef);	 
		LCD_WRITE_REG(0x11); //Exit Sleep
		delay_ms(120);
		LCD_WRITE_REG(0x29); //display on	
    }
    LCD_DisplayDir(0);
    LCD_BlackLight(1);
    LCD_Clear(WHITE);
}

static void LCD_BlackLight(uint8_t status)
{
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, status); 
}


//д�Ĵ�������
//data:�Ĵ���ֵ
static void LCD_WRITE_REG(uint16_t data)
{
    LCD_RS_RESET();
    LCD_CS_RESET();
    LCD_WRITE_16BIT(data);
    LCD_WR_RESET();
    LCD_WR_SET();
    LCD_CS_SET();
    
}
//д���ݺ���
//data:����ֵ
static void LCD_WRITE_DATA(uint16_t data)
{
    LCD_RS_SET();
    LCD_CS_RESET();
    LCD_WRITE_16BIT(data);
    LCD_WR_RESET();
    LCD_WR_SET();
    LCD_CS_SET();  
}
//��LCD����
//����ֵ:������ֵ
static uint16_t LCD_RD_DATA(void)
{										   
	uint16_t t;
 	GPIOB->MODER=0X00000000; //PB0-15  ��������
	GPIOB->PUPDR=0X55555555; //
	GPIOB->ODR=0X0000;     //ȫ�����0

	LCD_RS_SET();
	LCD_CS_RESET();
	//��ȡ����(���Ĵ���ʱ,������Ҫ��2��)
	LCD_RD_RESET();
	if(LCD.id==0X8989)delay_ms(1);//FOR 8989,��ʱ2us					   
	t=LCD_READ_16BIT();  
	LCD_RD_SET();
	LCD_CS_SET(); 

	GPIOB->MODER=0X55555555; //PB0-15  �������
	GPIOB->PUPDR=0X55555555; // �������
	GPIOB->ODR=0XFFFF;    //ȫ�������
	return t;  
}

//д�Ĵ���
//LCD_Reg:�Ĵ������
//LCD_RegValue:Ҫд���ֵ
static void LCD_WriteReg(uint16_t LCD_Reg,uint16_t LCD_RegValue)
{	
	LCD_WRITE_REG(LCD_Reg);  
	LCD_WRITE_DATA(LCD_RegValue);	    		 
}
//���Ĵ���
//LCD_Reg:�Ĵ������
//����ֵ:������ֵ
static uint16_t LCD_ReadReg(uint16_t LCD_Reg)
{										   
 	LCD_WRITE_REG(LCD_Reg);  //д��Ҫ���ļĴ�����  
	return LCD_RD_DATA(); 
}

//��ʼдGRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WRITE_REG(LCD.WramCmd);
} 

//����LCD���Զ�ɨ�跽��
//ע��:�����������ܻ��ܵ��˺������õ�Ӱ��(������9341/6804����������),
//����,һ������ΪL2R_U2D����,�������Ϊ����ɨ�跽ʽ,���ܵ�����ʾ������.
//dir:0~7,����8������(���嶨���lcd.h)
//9320/9325/9328/4531/4535/1505/b505/5408/9341/5310/5510/1963��IC�Ѿ�ʵ�ʲ���	   	   
void LCD_Scan_Dir(uint8_t dir)
{
	uint16_t regval=0;
	uint16_t dirreg=0;
	uint16_t temp;  
	if((LCD.dir==1&&LCD.id!=0X6804&&LCD.id!=0X1963)||(LCD.dir==0&&LCD.id==0X1963))//����ʱ����6804��1963���ı�ɨ�跽������ʱ1963�ı䷽��
	{			   
		switch(dir)//����ת��
		{
			case 0:dir=6;break;
			case 1:dir=7;break;
			case 2:dir=4;break;
			case 3:dir=5;break;
			case 4:dir=1;break;
			case 5:dir=0;break;
			case 6:dir=3;break;
			case 7:dir=2;break;	     
		}
	} 
	if(LCD.id==0x9341||LCD.id==0X6804||LCD.id==0X5310||LCD.id==0X5510||LCD.id==0X1963)//9341/6804/5310/5510/1963,���⴦��
	{
		switch(dir)
		{
			case L2R_U2D://������,���ϵ���
				regval|=(0<<7)|(0<<6)|(0<<5); 
				break;
			case L2R_D2U://������,���µ���
				regval|=(1<<7)|(0<<6)|(0<<5); 
				break;
			case R2L_U2D://���ҵ���,���ϵ���
				regval|=(0<<7)|(1<<6)|(0<<5); 
				break;
			case R2L_D2U://���ҵ���,���µ���
				regval|=(1<<7)|(1<<6)|(0<<5); 
				break;	 
			case U2D_L2R://���ϵ���,������
				regval|=(0<<7)|(0<<6)|(1<<5); 
				break;
			case U2D_R2L://���ϵ���,���ҵ���
				regval|=(0<<7)|(1<<6)|(1<<5); 
				break;
			case D2U_L2R://���µ���,������
				regval|=(1<<7)|(0<<6)|(1<<5); 
				break;
			case D2U_R2L://���µ���,���ҵ���
				regval|=(1<<7)|(1<<6)|(1<<5); 
				break;	 
		}
		if(LCD.id==0X5510)dirreg=0X3600;
		else dirreg=0X36;
 		if((LCD.id!=0X5310)&&(LCD.id!=0X5510)&&(LCD.id!=0X1963))regval|=0X08;//5310/5510/1963����ҪBGR   
		if(LCD.id==0X6804)regval|=0x02;//6804��BIT6��9341�ķ���	   
		LCD_WriteReg(dirreg,regval);
		if(LCD.id!=0X1963)//1963�������괦��
		{
			if(regval&0X20)
			{
				if(LCD.width<LCD.height)//����X,Y
				{
					temp=LCD.width;
					LCD.width=LCD.height;
					LCD.height=temp;
				}
			}else  
			{
				if(LCD.width>LCD.height)//����X,Y
				{
					temp=LCD.width;
					LCD.width=LCD.height;
					LCD.height=temp;
				}
			}  
		}
		if(LCD.id==0X5510)
		{
			LCD_WRITE_REG(LCD.SetXPosCmd);LCD_WRITE_DATA(0); 
			LCD_WRITE_REG(LCD.SetXPosCmd+1);LCD_WRITE_DATA(0); 
			LCD_WRITE_REG(LCD.SetXPosCmd+2);LCD_WRITE_DATA((LCD.width-1)>>8); 
			LCD_WRITE_REG(LCD.SetXPosCmd+3);LCD_WRITE_DATA((LCD.width-1)&0XFF); 
			LCD_WRITE_REG(LCD.SetYPosCmd);LCD_WRITE_DATA(0); 
			LCD_WRITE_REG(LCD.SetYPosCmd+1);LCD_WRITE_DATA(0); 
			LCD_WRITE_REG(LCD.SetYPosCmd+2);LCD_WRITE_DATA((LCD.height-1)>>8); 
			LCD_WRITE_REG(LCD.SetYPosCmd+3);LCD_WRITE_DATA((LCD.height-1)&0XFF);
		}else
		{
			LCD_WRITE_REG(LCD.SetXPosCmd); 
			LCD_WRITE_DATA(0);LCD_WRITE_DATA(0);
			LCD_WRITE_DATA((LCD.width-1)>>8);LCD_WRITE_DATA((LCD.width-1)&0XFF);
			LCD_WRITE_REG(LCD.SetYPosCmd); 
			LCD_WRITE_DATA(0);LCD_WRITE_DATA(0);
			LCD_WRITE_DATA((LCD.height-1)>>8);LCD_WRITE_DATA((LCD.height-1)&0XFF);  
		}
  	}else 
	{
		switch(dir)
		{
			case L2R_U2D://������,���ϵ���
				regval|=(1<<5)|(1<<4)|(0<<3); 
				break;
			case L2R_D2U://������,���µ���
				regval|=(0<<5)|(1<<4)|(0<<3); 
				break;
			case R2L_U2D://���ҵ���,���ϵ���
				regval|=(1<<5)|(0<<4)|(0<<3);
				break;
			case R2L_D2U://���ҵ���,���µ���
				regval|=(0<<5)|(0<<4)|(0<<3); 
				break;	 
			case U2D_L2R://���ϵ���,������
				regval|=(1<<5)|(1<<4)|(1<<3); 
				break;
			case U2D_R2L://���ϵ���,���ҵ���
				regval|=(1<<5)|(0<<4)|(1<<3); 
				break;
			case D2U_L2R://���µ���,������
				regval|=(0<<5)|(1<<4)|(1<<3); 
				break;
			case D2U_R2L://���µ���,���ҵ���
				regval|=(0<<5)|(0<<4)|(1<<3); 
				break;	 
		} 
		dirreg=0X03;
		regval|=1<<12; 
		LCD_WriteReg(dirreg,regval);
	}
}   


//����LCD��ʾ����
//dir:0,������1,����
static void LCD_DisplayDir(uint8_t dir)
{
    if(dir==0)
    {
        LCD.dir=0;
        LCD.height=LCD_HEIGHT;
        LCD.width=LCD_WIDTH;
        if(LCD.id==0x9341)
        {
            LCD.WramCmd = WRAM_CMD;
            LCD.SetXPosCmd = SET_X_POS_CMD;
            LCD.SetYPosCmd = SET_Y_POS_CMD;
        }
    }
    else
    {
        LCD.dir=1;
        LCD.height= LCD_WIDTH;
        LCD.width=LCD_HEIGHT;
        if(LCD.id==0x9341)
        {
            LCD.WramCmd = WRAM_CMD;
            LCD.SetXPosCmd = SET_X_POS_CMD;
            LCD.SetYPosCmd = SET_Y_POS_CMD;
        }
    }
    LCD_Scan_Dir(DFT_SCAN_DIR);	//Ĭ��ɨ�跽��
}
//���ù��λ��
//Xpos:������
//Ypos:������
static void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
    if(LCD.id == 0x9341)
    {
        LCD_WRITE_REG(LCD.SetXPosCmd); 
		LCD_WRITE_DATA(Xpos>>8);LCD_WRITE_DATA(Xpos&0XFF); 			 
		LCD_WRITE_REG(LCD.SetYPosCmd); 
		LCD_WRITE_DATA(Ypos>>8);LCD_WRITE_DATA(Ypos&0XFF); 	
    }
}
static void LCD_Clear(uint16_t color)
{
    uint32_t index=0;
    uint32_t totalPoint = LCD_WIDTH;
    totalPoint *= LCD_HEIGHT;
    LCD_SetCursor(0x00,0x00);
    LCD_WriteRAM_Prepare();
    for(index=0;index<totalPoint;index++)LCD_WRITE_DATA(color);	
}
//����
//x,y:����
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);		//���ù��λ�� 
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	LCD_WRITE_DATA(POINT_COLOR); 
}	 
//���ٻ���
//x,y:����
//color:��ɫ
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{	   
	if(LCD.id==0X9341||LCD.id==0X5310)
	{
		LCD_WRITE_REG(LCD.SetXPosCmd); 
		LCD_WRITE_DATA(x>>8);LCD_WRITE_DATA(x&0XFF);  			 
		LCD_WRITE_REG(LCD.SetYPosCmd); 
		LCD_WRITE_DATA(y>>8);LCD_WRITE_DATA(y&0XFF); 		 	 
	}
	LCD_RS_RESET();
 	LCD_CS_RESET(); 
	LCD_WRITE_16BIT(LCD.WramCmd);//дָ��  
	LCD_WR_RESET(); 
	LCD_WR_SET(); 
 	LCD_CS_SET(); 
	LCD_WRITE_DATA(color);		//д����
}


//��ָ��λ����ʾһ���ַ�
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{  							  
    uint8_t temp,t1,t;
	uint16_t y0=y;
	uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���	
 	num=num-' ';//�õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[num][t];	//����1608����
		else if(size==24)temp=asc2_2412[num][t];	//����2412����
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=LCD.height)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=LCD.width)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}
//��ʾ�ַ���
//x,y:�������
//width,height:�����С  
//size:�����С
//*p:�ַ�����ʼ��ַ		  
static void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p)
{         
	uint8_t x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//�˳�
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }  
}
//��ʾ�ַ���

//*p:�ַ�����ʼ��ַ		  
static void LCD_ShowStringLine(uint16_t row,uint16_t column,uint8_t *p)
{         
        LCD_ShowString(5+8*column,5+16*row,8*29,16,16,p);
}
static void LCD_ClearLine(uint16_t row)
{
    uint8_t *Display=(uint8_t *)"                                  ";
    LCD_ShowString(5,5+16*row,8*29,16,16,Display);
}

