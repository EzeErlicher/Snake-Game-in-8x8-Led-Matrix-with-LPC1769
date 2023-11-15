#include <LPC17xx.h>
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"

#define HARD_MAX    1800
#define NORMAL_MAX  3000

#define TIMER_EASY   2500
#define TIMER_NORMAL 1500
#define TIMER_HARD   800

volatile uint16_t adcValue = 0;

void configADC();        // Potenciometro para regular velocidad de juego
void configTimer();     //  Timer asociado al ADC
void stopGame();

int main(){
    configADC();
    configTimer();
    //ADC_StartCmd(LPC_ADC, ADC_START_NOW); //Hago una conversión antes de arrancar el juego
    while(1){}
    return 0;
}

void configADC(){
    PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;         //Sin pull-up ni pull-down
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);  //P0.23 como AD0.0

    ADC_Init(LPC_ADC, 200000);                       //Frec. de muestreo = 200kHz
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN0,ENABLE);      //Habilito interrupción canal 0
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0,ENABLE);    //Habilito canal
    NVIC_EnableIRQ(ADC_IRQn);                        //Habilito interrupción del ADC
    ADC_EdgeStartConfig(LPC_ADC,ADC_START_ON_RISING);//Selecciono los flancos de subida para iniciar la conversión
    //ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT10);       //Habilito la conversión por el Match 1.0
}

void configTimer(){
    //Usamos el Match 1.0 para comenzar la conversión del ADC en los flancos de subida
    //No hace falta cambiar el PINSEL

    TIM_TIMERCFG_Type TIMConfigStruct;
    TIMConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIMConfigStruct.PrescaleValue = 500000;
    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TIMConfigStruct); //Configuro el preescaler del timer 1 cada 500ms

    //Configuro el Match 1.0 para hacer un toggle cada 1seg
    TIM_MATCHCFG_Type MatchConfig;
	MatchConfig.MatchChannel = 0;
	MatchConfig.IntOnMatch = ENABLE;
	MatchConfig.ResetOnMatch = ENABLE;
	MatchConfig.StopOnMatch = DISABLE;
	MatchConfig.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	MatchConfig.MatchValue = 10;
	TIM_ConfigMatch(LPC_TIM1,&MatchConfig);
	NVIC_EnableIRQ(TIMER1_IRQn);

    TIM_Cmd(LPC_TIM1,ENABLE);
}

void ADC_IRQHandler(){
    //volatile uint16_t adcValue = 0; //Uso variable local, no hay necesidad de tenerla como global
	adcValue = 0;
    if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_0,ADC_DATA_DONE)){
		adcValue =  ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);
	}

    //A menor valor en la medición, mayor es la resistencia del potenciometro
    //Escala de mediociones del ADC: 0--(Zona dificil)--HARD_MAX--(Zona normal)--NORMAL_MAX--(Zona facil)--4095

    if(adcValue>NORMAL_MAX){        //El potenciometro está cerca de su valor minimo
        TIM_UpdateMatchValue(LPC_TIM0,0,TIMER_HARD);
    } else if(adcValue>HARD_MAX){   //El potenciometro está en un valor intermedio
        TIM_UpdateMatchValue(LPC_TIM0,0,TIMER_NORMAL);
    } else{                         //El potenciometro está cerca de su valor maximo
        TIM_UpdateMatchValue(LPC_TIM0,0,TIMER_EASY);
    }

    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
}

void TIMER1_IRQHandler(){
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
	TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);
}

void stopGame(){
    TIM_Cmd(LPC_TIM1,DISABLE);
    NVIC_DisableIRQ(ADC_IRQn);
}
