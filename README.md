# Snake Game
This repository showcases the implementation of the classic Snake game on a LPC1769 board (revB version), which contains an ARM Cortex-M3 processor. The game is displayed on an 8x8 LED matrix and has the following features:

● Speed Regulation: Game speed is controled through a potentiometer and the ADC module.

● Motion Control: Using either 4 physical buttons that trigger GPIO interruptions or through the "WASD" characters received via UART communication.

● Statistics Transmission: Statistics are sent through UART transmission at the end of each gaming session

● Sound: Upon finishing a game session, a 400Hz tone is reproduced. The samples of a the sinusoidal wave stored in memory are transmitted to the DAC thanks to a DMA channel

● Start and reset button 

List of components:

   - LPC1769 (revB version)
   - 8x8 LED matrix, model: KYX-1088AB
   - 8 resistors 430 Ὡ
   - 10KὩ potentiometer
   - 5 buttons
   - earphone for audio output
   - UART TTL to USB module type A PL2303
