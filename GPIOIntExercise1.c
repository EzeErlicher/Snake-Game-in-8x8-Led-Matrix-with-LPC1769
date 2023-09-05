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
void configOutputs(void);
void delay(uint32_t timeDelay);

uint8_t sequence1[]= {0,1,0,0,1,1,0,1,0};
uint8_t sequence2[]= {0,1,1,1,0,0,1,1,0};

int main(void) {

	configGPIOInterrupts();
	configOutputs();


	while(1){

		LPC_GPIO0->FIOSET0=0x03;
		// while no interrupt occurs, P0.0 and P0.1 are set on 1
	}

    return 0 ;
}

void EINT3_IRQHandler(void){

	// if a rising edge interrupt occurs on P2.0
	if(LPC_GPIOINT->IO2IntStatR & (1<<0)){

		for(uint8_t i=0;i<sizeof(sequence1)/sizeof(sequence1[0]);i++){

			if(sequence1[i]==0){ LPC_GPIO0->FIOCLR0=0x01; }
			else{ LPC_GPIO0->FIOSET0=0x01; }
			delay(1000);
		}

		LPC_GPIOINT->IO2IntClr=0x00000001;
	}

    // if an interrupt occurs on P2.1
	else{

		for(uint8_t i=0;i<sizeof(sequence2)/sizeof(sequence2[0]);i++){

			if(sequence2[i]==0){ LPC_GPIO0->FIOCLR0=0x02; }
			else{ LPC_GPIO0->FIOSET0=0x02; }
			delay(1000);
		}
		LPC_GPIOINT->IO2IntClr=0x00000002;

	}
}


void configGPIOInterrupts(void){

	LPC_PINCON->PINSEL4 &= 0xFFFFFFF0;   //  configures P2.0 and P2.1 as GPIO'S
	LPC_GPIO2->FIODIRL &=  0xFFFC;       //  configures p2.0 and p2.1 as inputs
	// pull-up resistors enabled by default

	LPC_GPIOINT->IO2IntEnR |=0x00000001; // enables rising edge interrupt on P2.0
	LPC_GPIOINT->IO2IntClr = 0x00000001; // clears interrupt flag for P2.0

	LPC_GPIOINT->IO2IntEnF |=0x00000002; // enables falling edge interrupt on P2.1
	LPC_GPIOINT->IO2IntClr = 0x00000002; // clears interrupt flag for P2.1

	NVIC_EnableIRQ(EINT3_IRQn);

}

void configOutputs(void){

	LPC_PINCON->PINSEL0 &=0xFFFFFFF0; // P0.0 and P0.1 as GPIO
	LPC_GPIO0->FIODIR |= 0x00000003;  // P0.0 and P0.1 as outputs

}

void delay(uint32_t timeDelay){

	for(uint32_t i=0; i<timeDelay; i++)
			for(uint32_t j=0; j<timeDelay; j++);

}

