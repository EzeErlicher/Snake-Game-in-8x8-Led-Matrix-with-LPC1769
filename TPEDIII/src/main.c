#include <LPC17xx.h>
#include <string.h>
#include "LPC17xx.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"

uint16_t X[0x0020,0x0001,0x0002,0x0008,0x0004,0x0010,0x0040,0x0080]
uint16_t Y[0x0DF0,0x07F0,0x0EF0,0x0BF0,0x0FE0,0x0F70,0x0FD0,0x0FB0]

#define ANCHO 8
#define ALTO 8

uint8_t  actualX=0,
             actualY=0xF;
    uint16_t FIOX=0,
             FIOY=0xF;

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

volatile uint16_t secondsCounter = 0;


void configPulsadores(); // Interrupciones Externas
void configTimers();     // Tick para mover la vibora
void configADC();        // Potenciometro para regular velocidad de juego
void configDAC();        // Salida de sonido para WIN/GameOver
void configGPIO();       // Matriz Led
void configUART();       // Envio de estadisticas
void configSysTick();    // Obtención de seeds para generar random


uint8_t checkCollisions(Point newPos);
void updateDirection(Direction new, Direction avoid);
void createNewApple();
void moveSnake();
//Chequea y envía los leds a encender a la matriz via I2P
void render();
/*  Lee el valor de conversión del ADC y asigna uno de los tres niveles de velocidad
    de juego seteando el timer principal acordemente
*/
void setDifficulty(uint16_t value);
void sendStats();
void initGame();
//Detiene el juego, congelando el movimiento de la vibora
void stopGame();
void getRandomPair(uint8_t* a, uint8_t* b);

void delay(uint32_t times) {
	for(uint32_t i=0; i<times; i++)
		for(uint32_t j=0; j<times; j++);
}

int main() {
    initGame();
    //El juego no debería comenzar hasta que el jugador apriete uno de los pulsadores

    configGPIO();
    configSysTick();
    //configTimers();
    while (1) {
        moveSnake();
        render();
        delay(5000);
    }

    return 0;
}

//***********************************************
//              CONFIGURACIONES
//***********************************************

//Configura EINT0 (ARRIBA), EINT1 (ABAJO), EINT2 (IZQ), EINT3 (DER)

void configTimers(){
    //Configuro el timer 0 para que interrumpa cada 1ms
    TIM_TIMERCFG_Type TIMConfigStruct;
    TIMConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIMConfigStruct.PrescaleValue = 1000;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIMConfigStruct);

    TIM_MATCHCFG_Type MatchConfig;
    MatchConfig.MatchChannel = 0;
    MatchConfig.IntOnMatch = ENABLE;
    MatchConfig.ResetOnMatch = ENABLE;
    MatchConfig.StopOnMatch = DISABLE;
    MatchConfig.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    MatchConfig.MatchValue = 5000;
    TIM_ConfigMatch(LPC_TIM0,&MatchConfig);

    NVIC_EnableIRQ(TIMER0_IRQn);

    //TIM_Cmd(LPC_TIM0,ENABLE);
}

void configSysTick(){
    //Configura para que SysTick interrumpa cada 1ms
    SysTick_Config(SystemCoreClock / 1000);
}

void configGPIO(){

    // P0.4 hasta P0.11 como Outputs
	LPC_GPIO0->FIODIRL|=0x0FF0;
	LPC_GPIO0->FIOSET|=0xFFFF;

  // P2.0 hasta P0.7 como Outputs
	LPC_GPIO2->FIODIRL|=0x00FF;
	LPC_GPIO2->FIOCLRL|=0xFFFF;
}

void configUART(){
    //Configuro los pines Rx y Tx
    PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg); //P0.10
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg); //P0.11

    UART_CFG_Type UARTConfigStruct;           //Uso la configuración por defecto + baudRate=9600
    UARTConfigStruct.Baud_rate = 9600;
	UART_ConfigStructInit(&UARTConfigStruct); //Sin paridad, modo 5bits de ancho, 1bit de stop
	UART_Init(LPC_UART0, &UARTConfigStruct);


	UART_FIFO_CFG_Type UARTFIFOConfigStruct;  //Uso la configuración por defecto y desactivo DMA
    UARTFIFOConfigStruct.FIFO_DMAMode = DISABLE;
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct); //Reseteo ambos buffer, nivel de trigger 0
    UART_FIFOConfig(LPC_UART0, &UARTFIFOConfigStruct);
	//Habilito la transmisión
	UART_TxCmd(LPC_UART0, ENABLE);
}



//***********************************************
//                MECANICAS
//***********************************************

//Inicializa la vibora con tres leds de largo y su dirección + una manzana inicial
void initGame(){
    // Inicializa la longitud de la vibora y su dirección
    snakeLength = 3;
    direction = DER;

    snake[0].x = 0; snake[0].y = 4;
    snake[1].x = 1; snake[1].y = 4;
    snake[2].x = 2; snake[2].y = 4;

    apple.x=4;
    apple.y=6;
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
        //stopGame();
        //Sonido de GameOver
        //sendStats();
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
    char data[] = "Hola mi loco! Acá van los datos de la partida:\n\r";
    char data2[75]; //Reservo espacio suficiente para almacenar los datos de la partida como string

    sprintf(data2, "Cantidad de segundos jugados: %u - Manzanas comidas: %u\n\r", secondsCounter, appleCounter);

    UART_Send(LPC_UART0, (uint8_t*)data, sizeof(data), BLOCKING);
    UART_Send(LPC_UART0, (uint8_t*)data2, sizeof(data2), BLOCKING);

    return;
}

void render(){
     actualX=0;
     actualY=0xF;
     FIOX=0;
     FIOY=0xF;

    actualX |= (1<<apple.x);
    actualY |= (1<<apple.y);
    for(int i=0;i<snakeLength;i++){
        actualX |= (1<<snake[i].x);
        actualY |= ~(1<<snake[i].y);
    }

    for(int i=0;i<8;i++){
        if(actualX & 1<<i){
            FIOX |= X[i];
        }
        if(actualY & 1<<i){
            FIOY &= Y[i];
        }
    }

    LPC_GPIO2->FIOPINL = FIOX;
    LPC_GPIO0->FIOPINL = FIOY;
}

//***********************************************
//              INTERRUPCIONES
//***********************************************

//Lleva la cuenta de los segundos de la partida
void Systick_IRQHandler(){
    static uint8_t millisCount = 0;
	millisCount++;

    if(millisCount >= 1000){
	    secondsCounter++;
	    millisCount = 0;
	}
}

void TIMER0_IRQHandler(){
    moveSnake();
    render();
}




