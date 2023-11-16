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

#define HARD_MAX    3500
#define NORMAL_MAX  1000

#define TIMER_EASY   1300
#define TIMER_NORMAL 800
#define TIMER_HARD   400

#define samplesAmount 60
#define NUM_SINE_SAMPLE 60
#define SINE_FREQ_IN_HZ 400
#define PCLK_DAC_IN_MHZ 25

typedef struct {
    uint8_t x, y;
} Point;        //Ubicaciones dentro de la matriz 8x8

typedef enum {
    ARRIBA, ABAJO, IZQ, DER
} Direction;    //Direcciones en la que puede moverse la vibora

typedef enum {
    EASY, NORMAL, HARD
} Difficulty;

Point snake[ANCHO * ALTO];  // Arreglo de posiciones ocupadas por la vibora
uint8_t snakeLength;        // Cantidad de posiciones ocupadas por la vibora
Point apple;                // Ubicación actual de la manzana a comer
Direction direction;        // Direccion actual en la que se mueve la vibora
uint8_t appleCounter = 0;   // Cantidad de manzanas ya comidas
uint16_t secondsCounter;    // Duración de la partida en segundos
Difficulty difficulty;      // Dificultad actual de la partida

void configButtons();        // Interrupciones Externas
void configTimers();         // Tick para mover la vibora
void configADC();            // Potenciometro para regular velocidad de juego
void configDAC();            //
void configDMA_DAC_Channel();//
void configGPIO();           // Matriz Led
void configUART();           // Envio de estadisticas
void configSysTick();        // Obtención de seeds para generar random

uint8_t checkCollisions(Point newPos);
void updateDirection(Direction new, Direction avoid);
void createNewApple();
void moveSnake();
void render();
void helloWorld();
void sendStats();
void initGame();
void stopGame();
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
	for(uint8_t index = 0; index<NUM_SINE_SAMPLE; index++){
		sinSamples[index] = sinSamples[index]<<6;
    }
    
    configUART();
	configButtons();
    configGPIO();
    configADC();
    helloWorld();
    initGame();
    /*while (1) {}*/ //El juego no debería comenzar hasta que el jugador apriete uno de los pulsadores

    while (1) {
        render();
        delay(100); //SACAR CUENTAS
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
}

void configButtons(){
	LPC_PINCON->PINMODE0|=(3<<0);
	LPC_PINCON->PINMODE0|=(3<<2);
	LPC_PINCON->PINMODE0|=(3<<4);
	LPC_PINCON->PINMODE0|=(3<<6);

	//LPC_PINCON->PINMODE4|=(3<<26);
	LPC_PINCON->PINMODE1|=(3<<12);

	// Enable rising edge interrupt for pin
	LPC_GPIOINT->IO0IntEnR |=0x0000000F;
	LPC_GPIOINT->IO0IntEnR |=(1<<22);
	//LPC_GPIOINT->IO2IntEnR |=(1<<13);

	LPC_GPIOINT->IO0IntClr |=0x0000000F;
	LPC_GPIOINT->IO0IntClr |=(1<<22);
	//LPC_GPIOINT->IO2IntClr |=(1<<13);

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
	UART_ConfigStructInit(&UARTConfigStruct);   //Usamos la configuración por defecto
	UART_Init(LPC_UART1, &UARTConfigStruct);

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig(LPC_UART1, &UARTFIFOConfigStruct);

	UART_TxCmd(LPC_UART1, ENABLE);  //Habilitamos la transmisión
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
	LLI1.Control = samplesAmount | (1<<19)  | (1<<22) | (1<<26);

	//source width 32 bits dest width 32 bits source increment
	GPDMA_Init();

	GPDMA_Channel_CFG_Type GPDMACfg;
	GPDMACfg.ChannelNum = 0;
	GPDMACfg.SrcMemAddr = (uint32_t)sinSamples;
	GPDMACfg.DstMemAddr = 0;
	GPDMACfg.TransferSize = samplesAmount;
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
}

//***********************************************
//                MECANICAS
//***********************************************

//Inicializa todo lo necesario para una nueva partida
void initGame(){
    // Setea la longitud de la vibora y su dirección
    snakeLength = 3;
    direction = DER;
    appleCounter = 0;   //Resetea los contadores de manzanas comidas y segundos transcurridos
    secondsCounter =0;

    snake[0].x = 2; snake[0].y = 4;
    snake[1].x = 1; snake[1].y = 4;
    snake[2].x = 0; snake[2].y = 4;

    apple.x=6;
    apple.y=4;

    configTimers();
    ADC_StartCmd(LPC_ADC,ADC_START_NOW);    //Hace una unica conversión para obtener la velocidad de juego
    NVIC_EnableIRQ(ADC_IRQn);
    configSysTick();
    TIM_Cmd(LPC_TIM0,ENABLE);               //Habilita el timer encargado del tick de movimiento de la vibora
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
    }
}

/*  Chequea si el proximo movimiento newPos de la vibora contra cuatro situaciones:
    1) Fuera de los limites -> GameOver: Sonido + StopTotal + Enviar stats
    2) Choque contra si misma -> GameOver
    3) Choque con la manzana -> moveSnake + createNewApple + updateLength
    4) Espacio libre -> moveSnake
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
    static uint8_t	gameCounter = 0;    //Contador de partidas
    gameCounter++;
	uint8_t numbers[4]; //Buffer para el array de digitos

    uint8_t data0[] = "\n\rChan chan chan...Se terminó el juego mi loco! Acá van un par de estadisticas:\n\r";
    UART_Send(LPC_UART1,data0, sizeof(data0), BLOCKING);

    UART_Send(LPC_UART1,(uint8_t*)"	ID de partida: ",17, BLOCKING);
    uint16_to_uint8Array(gameCounter, numbers);
    UART_Send(LPC_UART1,(uint8_t *)numbers, sizeof(numbers), BLOCKING);

    UART_Send(LPC_UART1,(uint8_t*)"\n\r	Dificultad seleccionada: ",29, BLOCKING);
    if(difficulty==EASY){
        UART_Send(LPC_UART1,(uint8_t *)"FACIL",5,BLOCKING);
    } else if(difficulty==NORMAL){
        UART_Send(LPC_UART1,(uint8_t *)"NORMAL",6,BLOCKING);
    } else{
        UART_Send(LPC_UART1,(uint8_t *)"DIFICIL",7,BLOCKING);
    }
    UART_Send(LPC_UART1,(uint8_t*)"\n\r	Duración de la partida en segundos: ",41, BLOCKING);
    uint16_to_uint8Array(secondsCounter, numbers);
    UART_Send(LPC_UART1,(uint8_t *)numbers, sizeof(numbers), BLOCKING);

    UART_Send(LPC_UART1,(uint8_t*)"\n\r	Manzanas comidas: ",22, BLOCKING);
    uint16_to_uint8Array(appleCounter, numbers);
    UART_Send(LPC_UART1,numbers, sizeof(numbers), BLOCKING);

    UART_Send(LPC_UART1,(uint8_t *)"\n\r",2,BLOCKING);
}

//
void helloWorld(){
    UART_Send(LPC_UART1,(uint8_t*)"Bueeeenas! Gracias por jugar nuestro juego, acá te paso un par de tips sobre como funciona todo:\n\r", 100, BLOCKING);
    UART_Send(LPC_UART1,(uint8_t*)"  - Para iniciar la partida apretá el botón de Start/Restart\n\r", 65, BLOCKING);
    UART_Send(LPC_UART1,(uint8_t*)"  - Antes de iniciar cada partida vas a poder elejir la dificultad del juego con nuestro selector de velocidad\n\r", 113, BLOCKING);
    UART_Send(LPC_UART1,(uint8_t*)"  - Las reglas son bien simples: usá los botones de movimiento para comer todas las manzanas posibles sin chocarte con las paredes o tu propia cola\n\r", 151, BLOCKING);
    UART_Send(LPC_UART1,(uint8_t*)"  - Cuando pierdas (no te preocupes, en algún momento todos inevitablemente perdemos) te vamos a pasar algunas estadisticas y reproducir un sonido\n\r", 150, BLOCKING);
    UART_Send(LPC_UART1,(uint8_t*)"  - Pero eso no es todo! Queres seguir jugando? Simplemente presioná el botn de Start/Restart y probá tus habilidades de vuelta!!\n\r", 136, BLOCKING);
}

//Chequea y envía los leds a encender a la matriz
void render(){
    static uint16_t X[8]={0x0020,0x0001,0x0002,0x0008,0x0004,0x0010,0x0040,0x0080};
    static uint16_t Y[8]={0x0DF0,0x07F0,0x0EF0,0x0BF0,0x0FE0,0x0F70,0x0FD0,0x0FB0};

    static int i = 0;
    if(i>=(snakeLength+1)){
        i=0;
    }

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
*  - Detiene el timer0 (tick de movimiento)
*  - Detiene la interrupción del Systick (Contador de segundos)
*  - Configura y enciende el DAC con su canal de DMA correspondiente
*  - Envía las estadisticas de la partida por el puerto UART
*/
void stopGame(){
    TIM_Cmd(LPC_TIM0,DISABLE);
    SYSTICK_Cmd(DISABLE);
    sendStats();
    configDAC();
    configDMA_DAC_Channel();
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
}

//***********************************************
//              INTERRUPCIONES
//***********************************************

//Lleva la cuenta de los segundos de la partida
void SysTick_Handler(){
    static uint16_t millisCount = 0;
	millisCount++;

    if(millisCount >= 1000){
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
	else if((LPC_GPIOINT->IO0IntStatR)&(1<<3)){
		updateDirection(IZQ,DER);
		LPC_GPIOINT->IO0IntClr |=(1<<3);
	}
    // BOTON DE RESTART
	else{
        initGame();
		LPC_GPIOINT->IO0IntClr |=(1<<22);
	}
}

void ADC_IRQHandler(){
    __IO uint32_t adcValue = 0; //Uso variable local, no hay necesidad de tenerla como global
    if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_0,ADC_DATA_DONE)){ //Leo el valor de connversión en el canal 0
		adcValue =  ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);
	    NVIC_DisableIRQ(ADC_IRQn);
	}

    //A menor valor en la medición, mayor es la resistencia del potenciometro
    //Escala de mediociones del ADC: 0--(Zona dificil)--HARD_MAX--(Zona normal)--NORMAL_MAX--(Zona facil)--4095

    if(adcValue>HARD_MAX){        //El potenciometro está cerca de su valor minimo
        TIM_UpdateMatchValue(LPC_TIM0,0,TIMER_HARD);
        difficulty = HARD;
    } else if(adcValue>NORMAL_MAX){   //El potenciometro está en un valor intermedio
        TIM_UpdateMatchValue(LPC_TIM0,0,TIMER_NORMAL);
        difficulty = NORMAL;
    } else{                         //El potenciometro está cerca de su valor maximo
        TIM_UpdateMatchValue(LPC_TIM0,0,TIMER_EASY);
        difficulty = EASY;
    }

    LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
}

void TIMER1_IRQHandler(){
    NVIC_EnableIRQ(ADC_IRQn);
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
	TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);
}
