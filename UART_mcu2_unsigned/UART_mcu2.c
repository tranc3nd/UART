// main.c: starter code for CECS447 Project 2 Mode 2 for MCU to MCU communication
// Runs on TM4C123 

#include "tm4c123gh6pm.h"
#include "UART.h"
#include "PLL.h"
#include <stdint.h>
#include "Nokia5110.h"

// TODO: provide bit address definition for the three pins connected to the onboard LEDs
#define LED  (((volatile unsigned long)0x40025038))  // use onboard three LEDs: PF321

////////// Constants //////////  
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// white    RGB    0x0E
// pink     R-B    0x06
// Cran     -GB    0x0C

// TODO: finish the following constants definitions
#define DARK    		0X00
#define RED     		0X02
#define BLUE    		0X04
#define GREEN   		0X08
#define YELLOW  		0X0A
#define CYAN      	0X0C
#define WHITE   		0X0E
#define PURPLE  		0X06
#define NUM_COLORS  6

// TODO: define the constant for PORTF Enable bit in EN0 register
//#define NVIC_EN0_PORTF 

#define SW1					0x10
#define SW2					0x01

#define NVIC_EN0_PORTF 0x40000000  // enable PORTF edge interrupt

extern void DisableInterrupts(void);
extern void EnableInterrupts(void);
extern void WaitForInterrupt(void);  // low power mode

void GPIO_PortF_Init(void);
void Mode2(void);
void Mode3(void);


const unsigned long Color_Arry[] = {RED, GREEN, BLUE, PURPLE, WHITE, DARK};
unsigned char Mode;
unsigned char string[255];  // global to assist in debugging
volatile uint8_t color = 0;


// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(unsigned long ulCount){
  do{
    ulCount--;
	}while(ulCount);
}

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART1_OutChar(CR);
  UART1_OutChar(LF);
}

int main(void){
	unsigned char color;   // keep track of current LED color
  DisableInterrupts();	
	UART1_Init();							// initialize UART1
	PLL_Init();
	GPIO_PortF_Init();			  // initialize port F
	Nokia5110_Init();
	Nokia5110_Clear();
  EnableInterrupts();       
//	LEDs = DARK;
  while(1){
		Mode = UART1_InChar(); // busy waiting approach for UART Rx
//		LEDs = color;
		switch (Mode) {
			case '2': 
				Mode2(); 
				break;
			case '3': 
				Mode3(); 
				break;
			default:
				break;
		}
  }
}

void Mode2(void) { //MCU1 <-> MCU2
	GPIO_PORTF_DATA_R = UART1_InChar();
	while(Mode=='2') {
		WaitForInterrupt(); // saves power
	}
}

void Mode3(void) {
	Mode = '3';
	UART1_InString(string,255);
	Nokia5110_Clear();
	Nokia5110_OutString(string); // Copy message of strings to Nokia LCD screen
	while(Mode == '3') WaitForInterrupt();
}

void GPIO_PortF_Init(void)
{
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate clock for port F
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B; 			// unlock GPIO Port F
	GPIO_PORTF_CR_R |= 0x1F;      				// allow changes for PF4,PF0 take effect 
  GPIO_PORTF_AMSEL_R &= ~0x1F;  				// disable analog functionality on PF4-PF0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; 		// configure PF4-PF0 as GPIO
	GPIO_PORTF_DIR_R |= 0x0E;          // (c) make LEDs outputs
  GPIO_PORTF_DIR_R &= ~0x11;
  GPIO_PORTF_DEN_R |= 0x1F;     				// enable digital I/O on PF4-PF0
  GPIO_PORTF_AFSEL_R &= ~0x1F;  				// disable alt funct on PF4-PF0
  GPIO_PORTF_PUR_R |= 0x11;     				// enable weak pull-up on PF4-PF0
  GPIO_PORTF_IS_R &= ~0x11;     				// PF4-PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    				// PF4-PF0 is not both edges
  GPIO_PORTF_IEV_R |= 0x11;    				// PF4-PF0 rising edge event
  GPIO_PORTF_ICR_R = 0x11;      				// clear flags 4-0
  GPIO_PORTF_IM_R |= 0x11;      				// arm interrupt on PF4-PF0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF0FFFFF)|0x00A00000; // bits:23-21 for PORTF, set priority to 5
  NVIC_EN0_R |= NVIC_EN0_PORTF;      		// enable interrupt 30 in NVIC
}

void GPIOPortF_Handler(void)
{	
	if(Mode == '2') {
		// SW1 pressed changed color stay in mode2
		if(GPIO_PORTF_RIS_R&SW1) {
			Delay(83333);
			GPIO_PORTF_ICR_R = SW1;  // acknowledge flag
			color = (color + 1) % NUM_COLORS;
			GPIO_PORTF_DATA_R = Color_Arry[color];
		}
		
		// SW2 pressed pressed send color mode2 done
		if(GPIO_PORTF_RIS_R&SW2) {
			Delay(83333);
			GPIO_PORTF_ICR_R = SW2;  // acknowledge flag
			UART1_OutChar(Color_Arry[color]);
			Mode = '0';
		}
	}
	if(Mode == '3') {
		if(GPIO_PORTF_RIS_R&SW1) {
			Delay(83333);
			GPIO_PORTF_ICR_R = SW1;  // acknowledge flag
			UART1_OutChar('1');
			UART1_OutString((unsigned char *)"I recieved: ");
			UART1_OutString(string);
			UART1_OutChar(CR);
			Mode = '0';
		}
	}
}


