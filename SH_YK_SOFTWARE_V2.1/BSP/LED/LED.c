#include "LED.h"
				
void LED_GPIO_Config(void)	
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//DS_R
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOB, GPIO_Pin_15);	 
	
	//DS_G
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOC, GPIO_Pin_8);	\
	
	//DS_B
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOB, GPIO_Pin_0);	 
	
	//DS_Y
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOA, GPIO_Pin_1);	 
	
	//DS_W
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOA, GPIO_Pin_0);	 
}


void LEDW(u8 a)	
{
	     if(a==1)	  GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	else if (a==0)	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	else if (a==2)  GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0))));
}
void LEDY(u8 a)	
{
	  if(a==1)	    GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	  else if (a==0)GPIO_SetBits  (GPIOA,GPIO_Pin_1);
		else if (a==2)GPIO_WriteBit (GPIOA, GPIO_Pin_1, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1))));
}

void LEDB(u8 a)	
{
	  if(a==1)	    GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	  else if (a==0)GPIO_SetBits  (GPIOB,GPIO_Pin_0);
		else if (a==2)GPIO_WriteBit (GPIOB, GPIO_Pin_0, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0))));
}

void LEDG(u8 a)	
{
	  if(a==1)	    GPIO_ResetBits(GPIOC,GPIO_Pin_8);
	  else if (a==0)GPIO_SetBits  (GPIOC,GPIO_Pin_8);
		else if (a==2)GPIO_WriteBit (GPIOC,GPIO_Pin_8, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8))));
}


void LEDR(u8 a)	
{
	  if(a==1)	    GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	  else if (a==0)GPIO_SetBits(GPIOB,GPIO_Pin_15);
		else if (a==2)GPIO_WriteBit(GPIOB,GPIO_Pin_15, (BitAction)((1-GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_15))));
}


void USB_DET_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;    //œ¬¿≠ ‰»Î   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
}

uint8_t USB_DET(void) {return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_8));}


