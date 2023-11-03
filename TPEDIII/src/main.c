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


/*  Chequea si el proximo movimiento newPos de la vibora contra tres factores:
    - Fuera de los limites -> GameOver: Sonido + StopTotal + Enviar stats
    - Choque contra si misma -> GameOver
    - Choque con la manzana -> moveSnake + WIN: Sonido + newApple + updateLength
    - Espacio libre -> moveSnake
*/
void checkCollisions(Point newPos);

/*  Cuando el pulsador envia su dirección correspondiente, chequea si es valido
    y si es así actualiza la dirección actual de la vibora
*/
void updateMovimiento(Direction new, Direction avoid);

/*  Genera posición random para nueva manzana
    - Chequear si la posición está ocupada por la vibora
    - Update de la posición al elemento "apple"
*/
void newApple(); 

// Genera la nueva posición de la vibora y si es válida la actualiza en el arreglo snake
void moveSnake();

//Chequea y envía los leds a encender a la matriz via I2P
void render();

//Se encarga de enviar las estadisticas de la partida a la PC
void sendStats();

//Crea una vibora con dos leds de largo
void initGame();


int main() {
    

    while (1) {}

    return 0;
}
