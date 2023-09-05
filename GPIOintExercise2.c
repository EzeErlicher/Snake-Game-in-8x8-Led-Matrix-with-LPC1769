/*
 * Copyright 2022 NXP
 * NXP confidential.
 * This software is owned or controlled by NXP and may only be used strictly
 * in accordance with the applicable license terms.  By expressly accepting
 * such terms or by downloading, installing, activating and/or otherwise using
 * the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to
 * be bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software.
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>


void configGPIOInterrupts(void);

uint16_t aux=0;

int main(void) {


	configGPIOInterrupts();

    while(1) {

    }
    return 0 ;
}

void configGPIOInterrupts(void){

	LPC_PINCON->PINSEL0 &= 0xFFFFFFFC; //P0.0 as GPIO
	LPC_GPIO0->FIODIR &= 0xFFFFFFFE;   //P0.0 as input
	LPC_GPIOINT->IO0IntEnR |= 0x00000001; //P0.0 rising edge interrupt enabled
	LPC_GPIOINT->IO0IntClr =0x00000001; // P0.0 interrupt flag cleared

	LPC_PINCON->PINSEL4 &= 0xFFFFFFFC; //P2.0 as GPIO
	LPC_GPIO2->FIODIR &= 0xFFFFFFFE;   //P2.0 as input
	LPC_GPIOINT->IO2IntEnR |= 0x00000001; //P2.0 rising edge interrupt enabled
	LPC_GPIOINT->IO2IntClr = 0X00000001;  //P2.0 interrupt flag cleared


}

void EINT3_IRQHandler(void){


	if(LPC_GPIOINT->IntStatus & (1<<0)){

		aux=39;
		LPC_GPIOINT->IO0IntClr=0x00000001;
	}
	else{
		aux=29398;
		LPC_GPIOINT->IO2IntClr=0x00000001;
	}

}
