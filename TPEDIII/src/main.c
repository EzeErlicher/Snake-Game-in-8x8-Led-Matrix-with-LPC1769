#include <LPC17xx.h>
#include <string.h>
#include "LPC17xx.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_exti.h"

#define ANCHO 8
#define ALTO 8

#define HARD_MAX    1800
#define NORMAL_MAX  3000

#define TIMER_EASY   2500
#define TIMER_NORMAL 1500
#define TIMER_HARD   800

#define DMA_SIZE 60
#define NUM_SINE_SAMPLE 60
#define SINE_FREQ_IN_HZ 100
#define PCLK_DAC_IN_MHZ 25

uint16_t secondsCounter = 0;
volatile uint16_t adcValue = 0;

typedef struct {
    uint8_t x, y;
} Point;        //Ubicaciones dentro de la matriz 8x8

typedef enum {
    ARRIBA, ABAJO, IZQ, DER
} Direction;    //Direcciones en la que puede moverse la vibora

Point snake[ANCHO * ALTO];  //Arreglo de posiciones ocupadas por la vibora
uint8_t snakeLength;        //Cantidad de posiciones ocupadas por la vibora
Point apple;                //Ubicación actual de la manzana a comer
Direction direction;        //Direccion actual en la que se mueve la vibora
uint8_t appleCounter = 0;   //Cantidad de manzanas ya comidas




void configButtons(); // Interrupciones Externas
void configTimers();     // Tick para mover la vibora
void configADC();        // Potenciometro para regular velocidad de juego
void configDAC();            // Salida de sonido para WIN/GameOver
void configDMA_DAC_Channel();
void configGPIO();       // Matriz Led
void configUART();       // Envio de estadisticas
void configSysTick();    // Obtención de seeds para generar random


uint8_t checkCollisions(Point newPos);
void updateDirection(Direction new, Direction avoid);
void createNewApple();
void moveSnake();
void render();
void sendStats();
void initGame();
void stopGame();
//Resetea el juego
void resetGame();
void getRandomPair(uint8_t* a, uint8_t* b);
void uint16_to_uint8Array(uint16_t value, uint8_t *result);

void delay(uint32_t times) {
	for(uint32_t i=0; i<times; i++)
		for(uint32_t j=0; j<times; j++);
}

uint32_t sinSamples[NUM_SINE_SAMPLE] = {511, 564, 617, 669, 719, 767, 812, 853, 891, 925, 954, 978, 997, 1011, 1020, 1023,
							   1020, 1011, 997, 978, 954, 925, 891, 853, 812, 767, 719, 669, 617, 564, 511, 458,
							   405, 353, 303, 255, 210, 169, 131, 97, 68, 44, 25, 11, 2, 0, 2, 11, 25, 44, 68,
							   97, 131, 169, 210, 255, 303, 353, 405, 458};

int main() {

	for(uint8_t index = 0; index<NUM_SINE_SAMPLE; index++)
			sinSamples[index] = sinSamples[index]<<6;

    configButtons();
    configGPIO();
    configSysTick();
    configTimers();
    configUART();
    initGame();
    configDAC();
    configDMA_DAC_Channel();
    /*while (1) {}*/ //El juego no debería comenzar hasta que el jugador apriete uno de los pulsadores

    TIM_Cmd(LPC_TIM0,ENABLE);
    while (1) {
        render();
        //moveSnake();
        //render();
        delay(100);
    }

    return 0;
}

//***********************************************
//              CONFIGURACIONES
//***********************************************


void configTimers(){
	TIM_MATCHCFG_Type MatchConfig;
	MatchConfig.MatchChannel = 0;
	MatchConfig.IntOnMatch = ENABLE;
	MatchConfig.ResetOnMatch = ENABLE;
	MatchConfig.StopOnMatch = DISABLE;
	MatchConfig.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	MatchConfig.MatchValue = 2000;
	TIM_ConfigMatch(LPC_TIM0,&MatchConfig);

    //Configura el prescaler para que incremente el contador cada 1ms
    TIM_TIMERCFG_Type TIMConfigStruct;
    TIMConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIMConfigStruct.PrescaleValue = 1000;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIMConfigStruct);
    NVIC_EnableIRQ(TIMER0_IRQn);
    //TIM_Cmd(LPC_TIM0,ENABLE);         //No lo prendo aún
    /*************************************************/

    //Usamos el Match 1.0 para iniciar la conversión del ADC en cada interrupción
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
	MatchConfig.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	MatchConfig.MatchValue = 10;
	TIM_ConfigMatch(LPC_TIM1,&MatchConfig);
	NVIC_EnableIRQ(TIMER1_IRQn);
    TIM_Cmd(LPC_TIM1,ENABLE);

}

void configButtons(){

	LPC_PINCON->PINMODE0|=(3<<0);
	LPC_PINCON->PINMODE0|=(3<<2);
	LPC_PINCON->PINMODE0|=(3<<4);
	LPC_PINCON->PINMODE0|=(3<<6);

	// Enable rising edge interrupt for pin
	LPC_GPIOINT->IO0IntEnR |=0x0000000F;

	LPC_GPIOINT->IO0IntClr |=0x0000000F;

	// Enable EINT3 interrupt
	NVIC_EnableIRQ(EINT3_IRQn);
}

void configSysTick(){
    //Configura para que SysTick interrumpa cada 1ms
    SysTick_Config(SystemCoreClock/1000);
    SYSTICK_Cmd(ENABLE);
    SYSTICK_IntCmd(ENABLE);
}

void configGPIO(){

    // P0.4 hasta P0.11 como Outputs
	LPC_GPIO0->FIODIRL|=0x0FF0;
	LPC_GPIO0->FIOSETL|=0xFFFF;

  // P2.0 hasta P2.7 como Outputs
	LPC_GPIO2->FIODIRL|=0x00FF;
	LPC_GPIO2->FIOCLRL|=0xFFFF;
}

void configUART(){
    //Configuro los pines Rx y Tx
    PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 15;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg); //P0.15
	PinCfg.Pinnum = 16;
	PINSEL_ConfigPin(&PinCfg); //P0.16

	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
		//configuraci n por defecto:
	UART_ConfigStructInit(&UARTConfigStruct);
		//inicializa perif rico
	UART_Init(LPC_UART1, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
		//Inicializa FIFO
	UART_FIFOConfig(LPC_UART1, &UARTFIFOConfigStruct);
		//Habilita transmisi n
	UART_TxCmd(LPC_UART1, ENABLE);
}

void configDAC(){

	PINSEL_CFG_Type pinCfg;
	pinCfg.Funcnum = 2;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = 0;
	pinCfg.Portnum = 0;
	pinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&pinCfg);

	DAC_CONVERTER_CFG_Type dacCfg;
	dacCfg.CNT_ENA = SET;
	dacCfg.DMA_ENA = SET;
	DAC_Init(LPC_DAC);
		/*Set timeout*/
	uint32_t tmp;
	tmp = (PCLK_DAC_IN_MHZ * 1000000)/(SINE_FREQ_IN_HZ * NUM_SINE_SAMPLE);
	DAC_SetDMATimeOut(LPC_DAC, tmp);
	DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
}

void configDMA_DAC_Channel(){

	GPDMA_LLI_Type LLI1;
	LLI1.SrcAddr = (uint32_t) sinSamples;
	LLI1.DstAddr = (uint32_t) &LPC_DAC->DACR;
	LLI1.NextLLI = (uint32_t) &LLI1;
	LLI1.Control = DMA_SIZE
					   | (2<<18) //source width 32 bits
					   | (2<<21) //dest width 32 bits
					   | (1<<26); //source increment

	GPDMA_Init();

	GPDMA_Channel_CFG_Type GPDMACfg;
	GPDMACfg.ChannelNum = 0;
	GPDMACfg.SrcMemAddr = (uint32_t)sinSamples;
	GPDMACfg.DstMemAddr = 0;
	GPDMACfg.TransferSize = DMA_SIZE;
	GPDMACfg.TransferWidth = 0;
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	GPDMACfg.SrcConn = 0;
	GPDMACfg.DstConn = GPDMA_CONN_DAC;
	GPDMACfg.DMALLI = (uint32_t)&LLI1;
	GPDMA_Setup(&GPDMACfg);
	GPDMA_ChannelCmd(0, ENABLE);

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

//***********************************************
//                MECANICAS
//***********************************************

//Inicializa la vibora con tres leds de largo y su dirección + una manzana inicial
void initGame(){
    // Inicializa la longitud de la vibora y su dirección
    snakeLength = 3;
    direction = DER;

    snake[0].x = 2; snake[0].y = 4;
    snake[1].x = 1; snake[1].y = 4;
    snake[2].x = 0; snake[2].y = 4;

    apple.x=6;
    apple.y=4;
}

// Genera la nueva posición de la vibora y si es válida la actualiza en el arreglo snake
void moveSnake(){
    Point newPos = snake[0]; //Copia de la posición actual de la cabeza de la vibora
    if(direction==ARRIBA){
        newPos.y++;
    } else if(direction==ABAJO){
        newPos.y--;
    } else if(direction==DER){
        newPos.x++;
    } else{     //direction==IZQ
        newPos.x--;
    }

    if(!checkCollisions(newPos)){ //Chequeo si estoy haciendo un movimiento válido
        for(int i=snakeLength-1;i>0;i--){ //Recorro el arreglo en orden inverso
            snake[i]=snake[i-1];    //Muevo las posiciones de la vibora un lugar a la derecha dentro del array
        }
        snake[0]=newPos; //Guardo la nueva posición de la cabeza de la vibora
    } else{
        stopGame();
        //Sonido de GameOver
        sendStats();
        configDAC();            // Salida de sonido para WIN/GameOver
        configDMA_DAC_Channel();
    }
}

/*  Chequea si el proximo movimiento newPos de la vibora contra cuatro situaciones:
    - Fuera de los limites -> GameOver: Sonido + StopTotal + Enviar stats
    - Choque contra si misma -> GameOver
    - Choque con la manzana -> moveSnake + WIN: Sonido + createNewApple + updateLength
    - Espacio libre -> moveSnake
    ---
       Return: -1 = Game Over // 0 = moveSnake
*/
uint8_t checkCollisions(Point newPos){
    //Caso 1: Fuera de los limites
    if(newPos.x>=ANCHO || newPos.x<0 || newPos.y>=ALTO || newPos.y<0){
        return -1;
    }
    //Caso 2: Choque consigo misma
    for (int i = 0; i < snakeLength; i++){
        if(newPos.x==snake[i].x && newPos.y==snake[i].y){
            return -1;
        }
    }
    //Caso 3: Choque con una manzana
    if (newPos.x==apple.x && newPos.y==apple.y){
        snakeLength++;
        appleCounter++;
        createNewApple();
        //Sonido de WIN
        return 0;
    }
    //Caso 4: Movimiento válido a espacio vacío
    return 0;
}

/*  Cuando el pulsador envia su dirección correspondiente, chequea si es valido
    y si es así actualiza la dirección actual de la vibora
*/
void updateDirection(Direction new, Direction avoid){
    if(direction!= new && direction!=avoid){
        direction=new;
    }
}

/*  Genera posición random para nueva manzana
    - Chequea si la posición está ocupada por la vibora
    - Update de la posición al elemento "apple"
*/
void createNewApple(){
    Point newApple;
    uint8_t flag = 1;
    while(flag!=0){
        flag = 0;
        getRandomPair(&newApple.x,&newApple.y);
        for(int i=0;i<snakeLength;i++){ //Verifico la posición de newApple contra todas las de la vibora
            if(snake[i].x==newApple.x && snake[i].y==newApple.y){
                flag++; //Si encuentra una coincidencia, levanto la bandera
            }
        }
    }
    apple=newApple; //Guardo la nueva posición
}

//Usa el value de Systick para generar dos valores en el rango [0:7] que guarda en a y b
void getRandomPair(uint8_t* a, uint8_t* b){
    volatile uint32_t seed = SysTick->VAL;
    seed ^= (seed << 13);

    *a = (seed & 0x07);
    *b = ((seed >> 3) & 0x07);
}

//Se encarga de enviar las estadisticas de la partida a la PC
void sendStats(){
    uint8_t numbers[4], digits = 0;  //

    uint8_t data1[] = "Hola mi loco! Acá van los datos de la partida:\n\r";
    UART_Send(LPC_UART1,data1, sizeof(data1), BLOCKING);
    char data2[]= "Cantidad de segundos jugados: ";
    UART_Send(LPC_UART1,(uint8_t *)data2, sizeof(data2), BLOCKING);
    uint16_to_uint8Array(secondsCounter, numbers);
    UART_Send(LPC_UART1,(uint8_t *)numbers, sizeof(numbers), BLOCKING);
    char data3[]= " - Manzanas comidas: ";
    UART_Send(LPC_UART1,(uint8_t *)data3, sizeof(data3), BLOCKING);
    uint16_to_uint8Array(appleCounter, numbers);
    UART_Send(LPC_UART1,numbers, sizeof(numbers), BLOCKING);
    UART_Send(LPC_UART1,(uint8_t *)"\n\r\0",3,BLOCKING);

    return;
}

//Chequea y envía los leds a encender a la matriz
void render(){
    static uint16_t X[8]={0x0020,0x0001,0x0002,0x0008,0x0004,0x0010,0x0040,0x0080};
    static uint16_t Y[8]={0x0DF0,0x07F0,0x0EF0,0x0BF0,0x0FE0,0x0F70,0x0FD0,0x0FB0};

    static int i = 0;
    if(i>=(snakeLength+1)){
        i=0;
    }
    uint8_t actualX=0;      //Acumulador de flags de las cordenadas en X a encender
    uint8_t actualY=0;   //Acumulador de flags de las cordenadas en Y a encender
    uint16_t FIOX=0;        //Acumulador de pines a encender en X
    uint16_t FIOY=0xFFFF;   //Acumulador de pines a encender en Y

    if(!i){     //Renderizar posición de la manzana
        LPC_GPIO2->FIOPINL = X[apple.x];
        LPC_GPIO0->FIOPINL = Y[apple.y];
    } else{
        LPC_GPIO2->FIOPINL = X[snake[i-1].x];
        LPC_GPIO0->FIOPINL = Y[snake[i-1].y];
    }
    i++;
}

/* Detiene el juego, congelando el movimiento de la vibora
*  - Detiene el timer0 (tick del sistema)
*  -
*/
void stopGame(){
    TIM_Cmd(LPC_TIM0,DISABLE);
    SYSTICK_Cmd(DISABLE);
    TIM_Cmd(LPC_TIM1,DISABLE);
    NVIC_DisableIRQ(ADC_IRQn);
}

//Convierte un entero de 16bits a un string
void uint16_to_uint8Array(uint16_t value, uint8_t *result){
// Buffer size based on the maximum number of digits in a uint16_t (5 digits)
    uint8_t buffer[5];

    // Initialize index
    int8_t index = 0;

    // Handle the case when the value is 0 separately
    if (value == 0) {
        buffer[index++] = '0';
    } else {
        // Extract digits in reverse order
        while (value > 0) {
            buffer[index++] = '0' + (value % 10);
            value /= 10;
        }
    }

    // Reverse the buffer to get the correct order
    for (int8_t i = 0; i < index; ++i) {
        result[i] = buffer[index - 1 - i];
    }

    // Null-terminate the result
    result[index] = '\0';

    return index;
}

//***********************************************
//              INTERRUPCIONES
//***********************************************

//Lleva la cuenta de los segundos de la partida
void SysTick_Handler(){
    static uint8_t millisCount = 0;
	millisCount++;

    if(millisCount >= 100){
	    secondsCounter++;
	    millisCount = 0;
	}
}

void TIMER0_IRQHandler(){
    moveSnake();
    TIM_ClearIntPending(LPC_TIM0,TIM_MR0_INT);
}

void EINT3_IRQHandler(){
	//ARRIBA
		if((LPC_GPIOINT->IO0IntStatR)&(1<<0)){
			updateDirection(DER,IZQ);
			LPC_GPIOINT->IO0IntClr |=(1<<0);
		}

		//DERECHA
		else if((LPC_GPIOINT->IO0IntStatR)&(1<<1)){
			updateDirection(ARRIBA,ABAJO);
			LPC_GPIOINT->IO0IntClr |=(1<<1);
		}

		//IZQUIERDA
		else if((LPC_GPIOINT->IO0IntStatR)&(1<<2)){
			updateDirection(ABAJO,ARRIBA);
			LPC_GPIOINT->IO0IntClr |=(1<<2);
		}

		//ABAJO
		else{
			updateDirection(IZQ,DER);
			LPC_GPIOINT->IO0IntClr |=(1<<3);
		}
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