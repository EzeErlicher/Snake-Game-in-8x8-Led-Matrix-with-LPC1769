# Snake Game
The project consisted in building the traditional Snake game using the LPC1769 board (revB version) wich contains an ARM Cortex-M3 processor. The game, displayed on an 8x8 LED matrix, has the following features:

● Speed regulator using a potentiometer and the ADC module 

● Motion control using 4 physical buttons that generate GPIO interruptions or through the "WASD" characters using UART reception. 

● Game instructions are displayed when the board is turned on and statistics are sent at the end of each game through UART transmission. 

● Reproduction of a tone through the DAC at the end of the game, which is carried out by transmitting memory data to the DAC using the DMA module.

● Start and reset button 

List of components:

   - LPC1769 (revB version)
   - 8x8 LED matrix model KYX-1088AB
   - 8 resistors 430 Ὡ
   - 10KὩ potentiometer
   - 5 buttons
   - earphone for audio output
   - UART TTL to USB module type A PL2303
