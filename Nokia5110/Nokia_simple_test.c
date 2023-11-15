// Nokia5110TestMain.c
// Runs on LM4F120/TM4C123
// Test the functions in Nokia5110.c by printing various
// things to the LCD.
// Daniel Valvano
// September 16, 2013
// Modified by Min He: migrate from keil4.7 to keil 5, simplified the code to show basic display.

// Font table, initialization, and other functions based
// off of Nokia_5110_Example from Spark Fun:
// 7-17-2011
// Spark Fun Electronics 2011
// Nathan Seidle
// http://dlnmh9ip6v2uc.cloudfront.net/datasheets/LCD/Monochrome/Nokia_5110_Example.pde

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// Blue Nokia 5110 (Adafruit)
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Ground        (Gnd, pin 1) ground
// 3.3V          (Vcc, pin 2) power
// SSI0Clk       (Clk, pin 3) connected to PA2
// SSI0Tx        (Din, pin 4) connected to PA5
// Data/Command  (DC,  pin 5) connected to PA6
// SSI0Fss       (CS,  pin 6) connected to PA3
// back light    (BL,  pin 7) not connected
// Reset         (RST, pin 8) connected to PA7

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected
// Ground        (Gnd, pin 8) ground

// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// SSI0Fss       (SCE, pin 3) connected to PA3
// Reset         (RST, pin 4) connected to PA7
// Data/Command  (D/C, pin 5) connected to PA6
// SSI0Tx        (DN,  pin 6) connected to PA5
// SSI0Clk       (SCLK, pin 7) connected to PA2
// back light    (LED, pin 8) not connected

#include "Nokia5110.h"
#include "PLL.h"

//  function delays 3*ulCount cycles
void Delay(unsigned long ulCount){
  do{
    ulCount--;
	}while(ulCount);
}

int main(void){
  unsigned short count = 0;
  PLL_Init();                           // set system clock to 50 MHz
  Nokia5110_Init();
  count = 0; 
  Nokia5110_Clear();
  Nokia5110_OutString("************* LCD Test *************Letter: Num:------- ---- ");
  Nokia5110_OutChar(127);               // print UT sign
  while(1){
    Nokia5110_SetCursor(5, 5);          // five leading spaces, bottom row
    Nokia5110_OutChar((count%26)+'A');
    Nokia5110_OutChar(' ');
    Nokia5110_OutUDec(count);
//    Delay(8333333);                     // delay ~0.5 sec at 50 MHz for launchpad
    Delay(833333);                     // delay ~0.5 sec at 50 MHz for simulation
    count = count + 1;
  }
}
