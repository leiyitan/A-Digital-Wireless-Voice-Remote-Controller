#include "BUTTON.h"

void BUTTON_Init(void){
  GPIO_InitTypeDef GPIO_InitStructure;
	//BT_1--PB13
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

	//BT_2--PB7
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//BT_3--PC0
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
	//BT_4--PB3
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//BT_5--PB14
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//BT_6--PC3
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
	//BT_7--PB5
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//BT_8-PC12
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
	//BT_9--
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);  
	
	//BT_10--PB4
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//BT_11--B12
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
		//BT_12--PC1
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
	//BT_13--PC2
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	//BT_14--PB9
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	//BT_15--PB8
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

//按键
uint8_t BT_1(void) {return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_13 ));}
uint8_t BT_2(void) {return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_7  ));}
uint8_t BT_3(void) {return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_0  ));}
uint8_t BT_4(void) {return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_3  ));}
uint8_t BT_5(void) {return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_14 ));}
uint8_t BT_6(void) {return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_3  ));}
uint8_t BT_7(void) {return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_5  ));}
uint8_t BT_8(void) {return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_12 ));}
uint8_t BT_9(void) {return  (GPIO_ReadInputDataBit(GPIOD ,GPIO_Pin_2  ));}
uint8_t BT_10(void){return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_4  ));}
uint8_t BT_11(void){return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_12 ));}
uint8_t BT_12(void){return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_1  ));}
uint8_t BT_13(void){return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_2  ));}
uint8_t BT_14(void){return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_9  ));}
uint8_t BT_15(void){return  (GPIO_ReadInputDataBit(GPIOB ,GPIO_Pin_8  ));}

//获取所有按键值
uint16_t BT_X(void){
return (BT_1() << 0)|\
	     (BT_2() << 1)|\
       (BT_3() << 2)|\
       (BT_4() << 3)|\
       (BT_5() << 4)|\
       (BT_6() << 5)|\
       (BT_7() << 6)|\
       (BT_8() << 7)|\
       (BT_9() << 8)|\
       (BT_10()<< 9)|\
       (BT_11()<<10)|\
       (BT_12()<<11)|\
       (BT_13()<<12)|\
       (BT_14()<<13)|\
       (BT_15()<<14);
}
