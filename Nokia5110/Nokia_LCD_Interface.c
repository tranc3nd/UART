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
#include "tm4c123gh6pm.h"
#include <stdint.h>

// Constants
#define NVIC_EN0_PORTF 0x40000000  // enable PORTF edge interrupt
#define LED  (((volatile unsigned long)0x40025038))
	
// Color    LED(s) PortF
// red      R--    0x02 0x1110
// blue     --B    0x04
// green    -G-    0x08
#define Red         0x02
#define Blue        0x04
#define Green       0x08
#define White       0x0E
#define Dark				0x00

#define SW1					0x10
#define SW2					0x01


// Function Prototypes
extern void DisableInterrupts(void);
extern void EnableInterrupts(void);
extern void WaitForInterrupt(void);

void SwitchLed_Init(void);

volatile uint8_t flag = 0;

//  function delays 3*ulCount cycles
void Delay(unsigned long ulCount){
  do{
    ulCount--;
	}while(ulCount);
}


// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay2(void){
	volatile uint32_t  time;
  time = 727240*20/91;  // 0.1sec
  while(time){
		time--;
  }
}

void debounce_button(void) {
		unsigned char j;
		for (j=0;j<8;j++) { 
			Delay(83333);
		}
}


int main(void){
	DisableInterrupts();
  PLL_Init();                           // set system clock to 50 MHz
  Nokia5110_Init();
	SwitchLed_Init();
  Nokia5110_Clear();
	Nokia5110_OutString("The System  is off.");
	EnableInterrupts();
  while(1){	
		WaitForInterrupt();
  }
}


void SwitchLed_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate clock for port F
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B; 			// unlock GPIO Port F
	GPIO_PORTF_CR_R |= 0x1F;      				// allow changes for PF4,PF0 take effect 
  GPIO_PORTF_AMSEL_R &= ~0x1F;  				// disable analog functionality on PF4-PF0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; 		// configure PF4-PF0 as GPIO
	GPIO_PORTF_DIR_R = 0x0E;    				// make PF4-0 in (built-in button)
  GPIO_PORTF_DEN_R |= 0x1F;     				// enable digital I/O on PF4-PF0
  GPIO_PORTF_AFSEL_R &= ~0x1F;  				// disable alt funct on PF4-PF0
  GPIO_PORTF_PUR_R |= 0x11;     				// enable weak pull-up on PF4-PF0
  GPIO_PORTF_IS_R &= ~0x11;     				// PF4-PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    				// PF4-PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    				// PF4-PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      				// clear flags 4-0
  GPIO_PORTF_IM_R |= 0x11;      				// arm interrupt on PF4-PF0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF0FFFFF)|0x00A00000; // bits:23-21 for PORTF, set priority to 5
  NVIC_EN0_R |= NVIC_EN0_PORTF;      		// enable interrupt 30 in NVIC
}

void GPIOPortF_Handler(void){
	if(GPIO_PORTF_RIS_R&SW2) {  // SW2 Cycle through LED's
		GPIO_PORTF_ICR_R = SW2;
		if(flag==1){
			Delay(83333);
			if(GPIO_PORTF_DATA_R&Red){
				GPIO_PORTF_DATA_R = Blue;
				Nokia5110_Clear();
				Nokia5110_OutString("The current LED is Blue");	
				Delay(83333);
			}else if(GPIO_PORTF_DATA_R&Blue){
				GPIO_PORTF_DATA_R = Green;
				Nokia5110_Clear();
				Nokia5110_OutString("The current LED is Green");
				Delay(83333);
			}else if(GPIO_PORTF_DATA_R&Green){
				GPIO_PORTF_DATA_R = Red;
				Nokia5110_Clear();
				Nokia5110_OutString("The current LED is Red");
				Delay(83333);
			}else{
				Nokia5110_Clear();
				Nokia5110_OutString("The current LED is Red");
				GPIO_PORTF_DATA_R = Red;
				Delay(83333);
			}
		}
  }
    
	if(GPIO_PORTF_RIS_R&SW1) {  // SW1 Toggle onboard LED
		debounce_button();
		GPIO_PORTF_ICR_R = SW1;
		//GPIO_PORTF_DATA_R ^= GPIO_PORTF_DATA_R;

		Nokia5110_Clear();
		if(GPIO_PORTF_DATA_R>0x01){ 
			Delay(183333);
			GPIO_PORTF_DATA_R = Dark;
			Nokia5110_Clear();
			Nokia5110_OutString("The System  is off.");
			flag = 0;
		}else{
			Delay(183333);
			GPIO_PORTF_DATA_R = Red;
			Nokia5110_OutString("The current LED is Red");
			flag = 1;
		}
		
  }

}




