#ifndef __LED_H
#define	__LED_H
#include "sys.h" 
void LED_GPIO_Config(void);
void LEDR(u8 a);
void LEDG(u8 a);
void LEDB(u8 a);
void LEDY(u8 a);
void LEDW(u8 a);

void USB_DET_Init(void);

uint8_t USB_DET(void);

#endif 

