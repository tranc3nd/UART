// main.c: starter code for CECS447 Project 2 Mode 2 for MCU to MCU communication
// Runs on TM4C123 

#include "tm4c123gh6pm.h"
#include "UART.h"
#include "PLL.h"
#include <stdint.h>
#include <stdio.h>

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
void Mode1(void);
void Mode2(void);
void Mode3(void);
void Display_Menu(void);


const unsigned long Color_Arry[] = {RED, GREEN, BLUE, PURPLE, WHITE, DARK};
unsigned char Mode;

unsigned char string[255];  // global to assist in debugging
unsigned char color = 0;

unsigned char prompt[] = "Please enter a color(r=RED, g=GREEN, b=BLUE, p=PURPLE, w=WHITE, d=DARK): ";
unsigned char redled[] = " Red LED is on.";
unsigned char blueled[] = " Blue LED  is on.";
unsigned char greenled[] = " Green LED is on.";
unsigned char purpleled[] = " Purple LED is on.";
unsigned char whiteled[] = " White LED is on.";
unsigned char darkled[] = " LED is off.";

unsigned char display1[] = "Welcome to CECS 447 Project 2 UART\n\r";
unsigned char display2[] = "	1. PC<->MCU_1\n\r";
unsigned char display3[] = "	2. MCU_1<->MCU_2 LED Control\n\r";
unsigned char display4[] = "	3. PC<->MCU_1<->MCU_2 Messenger\n\r";
unsigned char display5[] = "Please choose a communication mode (type 1 or 2 or 3 followed by return):\n\r";
unsigned char mode3[] = "Please enter a message (a string of characters): ";


// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
//  function delays 3*ulCount cycles
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
  UART0_OutChar(CR);
  UART0_OutChar(LF);
}

int main(void){
  DisableInterrupts();	
  UART0_Init();              // initialize UART0
	UART1_Init();							// initialize UART1
	PLL_Init();
	GPIO_PortF_Init();			  // initialize port F
  EnableInterrupts();       
  while(1){
		Display_Menu();

		switch (Mode) {
			case '1': 
				Mode1(); 
				break;
			case '2':
				GPIO_PORTF_DATA_R = Color_Arry[5]; // Start Mode 2 with no LED
				Mode2(); 
				break;
			case '3':
				GPIO_PORTF_DATA_R = Color_Arry[1]; // Start Mode 2 with Green LED
				Mode3();
				break;
			default:
				break;
		}
  }
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
}

void Mode1(void) { // MCU-> PC
		OutCRLF();
	  OutCRLF();
		UART0_OutString(prompt); // Goes to the PC serial terminal
		char C;
		C = UART0_InChar();
		switch(C) {
			case 'r': 
				GPIO_PORTF_DATA_R = RED;
				UART0_OutString(redled);
				OutCRLF();
				break;
			case 'b': 
				GPIO_PORTF_DATA_R = BLUE;		
				UART0_OutString(blueled);
				OutCRLF();
				break;
			case 'g': 
				GPIO_PORTF_DATA_R = GREEN;
				UART0_OutString(greenled);
				OutCRLF();
				break;
			case 'p': 
				GPIO_PORTF_DATA_R = PURPLE;
				UART0_OutString(purpleled);
				OutCRLF();
				break;
			case 'w': 
				GPIO_PORTF_DATA_R = WHITE;
				UART0_OutString(whiteled);
				OutCRLF();
				break;
			case 'd': 
				GPIO_PORTF_DATA_R = DARK;
				UART0_OutString(darkled);
				OutCRLF();
				break;
			default:
				GPIO_PORTF_DATA_R = DARK;
				break;
		}
		
}

void Mode2(void) { //MCU1 <-> MCU2
	Mode='2';
	UART1_OutChar('2'); // Sending to MCU2
	GPIO_PORTF_DATA_R = UART1_InChar();
	while(Mode=='2') {
		WaitForInterrupt(); // saves power
	}
}

void Mode3(void) {
	UART1_OutChar('3'); // Sending to MCU2
	OutCRLF();
	UART0_OutString(mode3);
	UART0_InString(string,255); // Recieve string from PC
	
	UART1_OutString(string); // Send message of string to MCU2
	UART1_OutChar(CR);
	
	unsigned char flag = UART1_InChar();
  while(flag != '1') {}
	UART1_InString(string,255); // Recieve ack string from MCU2
	OutCRLF();
	UART0_OutString(string);
	UART0_OutChar(CR);
	GPIO_PORTF_DATA_R = DARK;
	Mode = '0';
	flag = '0';
}

void Display_Menu(void) {
	OutCRLF();
	UART0_OutString(display1);
	UART0_OutString(display2);
	UART0_OutString(display3);
	UART0_OutString(display4);
	UART0_OutString(display5);
	Mode = UART0_InChar();
}




