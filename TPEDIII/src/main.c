#include <LPC17xx.h>

#define ANCHO  8
#define ALTO 8

typedef struct {
    int x, y;
} Point;
typedef enum {
    ARRIBA, ABAJO, IZQ, DER
} Direction;

Point snake[ANCHO * ALTO];
int snakeLength;
Point apple;
Direction direction;

void configPulsadores(); // Interrupciones Externas
void configTimers();     // Tick para mover la vibora + contador de segundos de juego?
void configADC();        // Potenciometro para regular velocidad de juego
void configDAC();        // Salida de sonido para WIN/GameOver
void configI2C();        // Comunicación con Matriz Led
void configEUART();      // Envio de estadisticas



uint8_t checkCollisions(Point newPos);
void updateDirection(Direction new, Direction avoid);

/*  Genera posición random para nueva manzana
    - Chequea si la posición está ocupada por la vibora
    - Update de la posición al elemento "apple"
*/
void newApple(); 
void moveSnake();

//Chequea y envía los leds a encender a la matriz via I2P
void render();

/*  Lee el valor de conversión del ADC y asigna uno de los tres niveles de velocidad 
    de juego seteando el timer principal acordemente
*/
void setDifficulty(uint16_t value);

//Se encarga de enviar las estadisticas de la partida a la PC
void sendStats();
void initGame();

//Detiene el juego
void stopGame();


int main() {
    initGame();
    //El juego no debería comenzar hasta que el jugador apriete uno de los pulsadores

    while (1) {}

    return 0;
}

//Inicializa la vibora con tres leds de largo y su dirección + una manzana inicial
void initGame(){
    // Inicializa la longitud de la vibora y su dirección
    snakeLength = 3;
    direction = DER;

    snake[0].x = 0; snake[0].y = 4;
    snake[1].x = 0; snake[1].y = 5;
    snake[2].x = 0; snake[2].y = 6;

    apple.x=4;
    apple.y=6;
}

// Genera la nueva posición de la vibora y si es válida la actualiza en el arreglo snake
void moveSnake(){   
    Point newPos = snake[0];
    if(direction==ARRIBA){
        newPos.y++;
    } else if(direction==ABAJO){
        newPos.y--;
    } else if(direction==DER){
        newPos.x++;
    } else{
        newPos.x--;
    }
    if(!checkCollisions(newPos)){ //Es un movimiento válido
        for(int i=snakeLength-1;i>0;i--){ //Muevo los puntos de la vibora un lugar a la derecha dentro del array
            snake[i]=snake[i-1];
        }
        snake[0]=newPos; //Guardo la nueva posición de la cabeza de la vibora en la posición 0
    } else{
        stopGame();
        //Sonido de GameOver
        sendStats();
    }
}

/*  Chequea si el proximo movimiento newPos de la vibora contra cuatro situaciones:
    - Fuera de los limites -> GameOver: Sonido + StopTotal + Enviar stats
    - Choque contra si misma -> GameOver
    - Choque con la manzana -> moveSnake + WIN: Sonido + newApple + updateLength
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
        newApple();
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

