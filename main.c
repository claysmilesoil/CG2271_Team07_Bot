/*----------------------------------------------------------------------------
 * HEADER FILES AND GLOBAL VARIABLES
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "stdbool.h"

#define PTC_RED 1

#define PTB0_Pin 0 // motor control pwm
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3

#define PTE29_Pin 29 // buzzer pwm
#define QUARTERBEAT 275
#define NOTEDELAY 30

#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
#define BAUD_RATE 9600

#define MASK(x) (1 << (x))

uint8_t rx_data = 0x00;

osSemaphoreId_t dataIn;

const osThreadAttr_t normal1 = {
	.priority = osPriorityNormal1
};

volatile bool isMoving = false;

typedef enum {OFF, ON} led_state;

static int green_pos[] = {7,0,3,4,5,6,10,11};

static uint32_t notes[] = {
        1430, // C4 - 262 Hz 0
        1274, // D4 - 294 Hz  1
        1135, // E4 - 330 Hz  2
        1073, // F4 - 349 Hz  3
        955, // G4 - 392 Hz  4
				851, // A4 - 440 Hz  5
				804, // A4s - 466 Hz  6
        758, // B4 - 494 Hz  7
				716, // C5 - 523 Hz 8
				676, // C5s - 554 Hz  9
				568, // E5 - 659 Hz  10
				357, // C6 - 1046 Hz 11
				0 // rest  12
};

static uint32_t intro[] = {
	2,12,2,12, 2,12,2,1, 2,2,12,2,2,12, 2,2,12,2,2,7,
	2,2,12,2,2,12, 2,2,12,2,2,7, 2,2,12,2,2,12, 2,2,12,2,2,9,
	2,2,7,2,2,7, 2,2,7,2,2,1, 2,2,7,2,2,7, 2,2,7,2,2,3,
	2,2,7,2,2,7, 2,2,7,2,2,1, 2,2,7,2,2,7, 2,2,8,2,2,9,
	10,2, 10,10,6,7,8,9 
};

static uint8_t introDuration[] = {
	2,2,2,2, 2,2,2,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	1,1, 1,2,4,4,4,4
};

static float durationChoose[] = {4, 2, 1, 0.5, 0.25};

/*----------------------------------------------------------------------------
 * INITIALIZATION & MISC FUNCTIONS
 *---------------------------------------------------------------------------*/
 
 void initGPIO() {
	 // enable clock
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	 
	// configure to GPIO 
	PORTC->PCR[PTC_RED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC_RED] |= PORT_PCR_MUX(1); 
	for (int i = 0; i < 8; i++) {
		PORTC->PCR[green_pos[i]] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[green_pos[i]] |= PORT_PCR_MUX(1);
	}
	 
	// set direction registers to output
	PTC->PDDR |= MASK(PTC_RED);
	for (int i = 0; i < 8; i++) {
		PTC->PDDR |= MASK(green_pos[i]);
	}	
	
	// turn off all GPIO initially
	PTC->PCOR = MASK(PTC_RED);
	for (int i = 0; i < 8; i++) {
		PTC->PCOR = MASK(green_pos[i]);
	}	
 }
 
 void initPWM() {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	 
	/*PORTB->PCR[18] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[18] |= PORT_PCR_MUX(1);
	PTB->PDDR |= MASK(18);
	PTB->PSOR = MASK(18); // for debug */
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK; // tpm1 ch0
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK; // tpm1 ch1
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK; // tpm2 ch0
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK; // tpm2 ch1
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);	 
	
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // clock source
	
	TPM1->MOD = 0; 
	TPM2->MOD = 0;
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0V = 0;
	TPM2_C1V = 0;
	
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
}
 
void initBuzzer(){
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch2
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);
	
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // clock source

	TPM0->MOD = 0;

	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2V = 0;
	
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
}
 
void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	// disable receive
	UART2->C2 &= ~(UART_C2_RE_MASK);
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// no parity check
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_RIE_MASK;
	// enable receive
	UART2->C2 |=(UART_C2_RE_MASK); 
}

void ledControl(int position, led_state s) {
	if (s == ON) {
		PTC->PSOR = MASK(position);
	} else {
		PTC->PCOR = MASK(position);
	}
}

/*----------------------------------------------------------------------------
 * TASKS
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
  for (;;) {
		osSemaphoreAcquire(dataIn, osWaitForever);
		if (rx_data == 0x00) {
			isMoving = false;
			TPM1_C0V = 0;
			TPM1_C1V = 0;
			TPM2_C0V = 0;
			TPM2_C1V = 0; // shld be minus the cnv by set speed unless underflow
		} else if (rx_data >= 0x01) {
			isMoving = true;
		}
	}
}

void tMotorControl (void *argument) {
  for (;;) {
		
	}
}

void tLEDGreen (void *argument) {
	for (;;) {
		if (isMoving) {
			for (int i = 0; i < 8; i++) {
				ledControl(green_pos[i], OFF);
			}
			for (int i = 0; i < 8; i++) {
				ledControl(green_pos[i], ON);
				osDelay(100);
				ledControl(green_pos[i], OFF);
			}
		} else {
			for (int i = 0; i < 8; i++) {
				ledControl(green_pos[i], ON);
			}
		}
	}
}

void tLEDRed (void *argument) {
  for (;;) {
		//red
		if(isMoving) {
			ledControl(PTC_RED, ON);
			osDelay(500);
			ledControl(PTC_RED,OFF);
			osDelay(500);
		} else {
			ledControl(PTC_RED, ON);
			osDelay(250);
			ledControl(PTC_RED,OFF);
			osDelay(250);
		}
	}
}

void tAudio (void *argument) {
	int length = sizeof(intro)/sizeof(intro[0]);
  for (;;) {
		for (int i = 0; i < length; i++) {
			TPM0->MOD = notes[intro[i]];
			TPM0_C2V = notes[intro[i]] / 2;
			osDelay(QUARTERBEAT*durationChoose[introDuration[i]]);
			TPM0->MOD = 0;
			TPM0_C2V = 0;
			osDelay(NOTEDELAY);
		}
		//TPM0->MOD = 851;
		//TPM0_C2V = 851/2;
	}
}
 
/*----------------------------------------------------------------------------
 * INTERRUPTS
 *---------------------------------------------------------------------------*/

void UART2_IRQHandler(){
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2 -> S1 & UART_S1_RDRF_MASK){
		rx_data = UART2 -> D;
		osSemaphoreRelease(dataIn);
	}
}

int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
	initGPIO();
	initPWM();
	initBuzzer();
	initUART2(BAUD_RATE);
	__enable_irq();
 
  osKernelInitialize();
	dataIn = osSemaphoreNew(1,0,NULL);
  osThreadNew(tBrain, NULL, &normal1);
	//osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLEDRed, NULL, NULL);
	osThreadNew(tLEDGreen, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);	
  osKernelStart();                      
  for (;;) {}
}
