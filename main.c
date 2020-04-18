/*----------------------------------------------------------------------------
 * HEADER FILES AND GLOBAL VARIABLES
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "stdbool.h"

/* Red LEDS pin */
#define PTC_RED 1

/* motor control PWM pins */
#define PTD0_Pin 0 // left front
#define PTD2_Pin 2
#define PTD3_Pin 3 // left back
#define PTD1_Pin 1
#define PTC8_Pin 8 // right front
#define PTD5_Pin 5
#define PTB0_Pin 0 // right back
#define PTB1_Pin 1

/* buzzer-related constants */
#define PTB2_Pin 2 // buzzer pwm
#define QUARTERBEAT 275
#define NOTEDELAY 30

/* UART-related constants */
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
#define BAUD_RATE 9600

/* MASKS MACRO */
#define MASK(x) (1 << (x))
#define MOVE_MASK(x) (x & 0x08)
#define DIRECTION_MASK(x) (x & 0x03)
#define START_MUSIC(x) (x & 0x80)
#define RUN_MUSIC(x) (x & 0x40)
#define END_MUSIC(x) (x & 0x20)
#define MOVE_FORWARD 0
#define MOVE_BACKWARD 1
#define MOVE_LEFT 2
#define MOVE_RIGHT 3

/* misc constants*/
#define LEFTDUTYCYCLE 0.8
#define RIGHTDUTYCYCLE 0.75

/* synchronization related variables */
#define MSG_COUNT 1

uint8_t rx_data = 0x00;

osSemaphoreId_t dataIn;

const osThreadAttr_t normal1 = {
	.priority = osPriorityNormal1
};

osMessageQueueId_t motorMsg;

/* other variables and enums */
volatile bool isMoving = false;
volatile bool isBackward = false;

typedef enum {OFF, ON} led_state;

/* green led pins, PORTC */
static int green_pos[] = {7,0,3,4,5,6,10,11};

/* tune related data */
static uint32_t notes[] = {
        1430, // C4 - 262 Hz 0
        1274, // D4 - 294 Hz  1
        1135, // E4 - 330 Hz  2
        1073, // F4 - 349 Hz  3
        955, // G4 - 392 Hz  4
				851, // A4 - 440 Hz  5
				804, // A4s - 466 Hz  6
        758, // B4 - 494 Hz  7
				716, // C5 - 523 Hz  8
				676, // C5s - 554 Hz  9
				568, // E5 - 659 Hz  10
				357, // C6 - 1046 Hz  11
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
 * INITIALIZATION
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
 
 void initMotors() {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch0, left front
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	PORTD->PCR[PTD2_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch2, left front
	PORTD->PCR[PTD2_Pin] |= PORT_PCR_MUX(4);	 
	PORTD->PCR[PTD3_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch3, left back
	PORTD->PCR[PTD3_Pin] |= PORT_PCR_MUX(4);	
	PORTD->PCR[PTD1_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch1, left back
	PORTD->PCR[PTD1_Pin] |= PORT_PCR_MUX(4);	
	PORTC->PCR[PTC8_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch4, right front
	PORTC->PCR[PTC8_Pin] |= PORT_PCR_MUX(3);	
 	PORTD->PCR[PTD5_Pin] &= ~PORT_PCR_MUX_MASK; // tpm0 ch5, right front
	PORTD->PCR[PTD5_Pin] |= PORT_PCR_MUX(4);		 
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK; // tpm1 ch0, right back
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK; // tpm1 ch1, right back
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);	 
	
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // clock source
	
	TPM0->MOD = 100; 
	TPM1->MOD = 100;
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(4));
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(4));
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	
	TPM0_C0V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
	TPM0_C1V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0;
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 
}
 
void initBuzzer(){
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// debug
	/*PORTB->PCR[18] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[18] |= PORT_PCR_MUX(1);
	PTB->PDDR |= MASK(18);
	PTB->PSOR = MASK(18); // for debug */
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK; // tpm2 ch0
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // clock source

	TPM2->MOD = 0;

	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0V = 0;
	
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // timer starts with this line
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

/*----------------------------------------------------------------------------
 * HELPER FUNCTIONS
 *---------------------------------------------------------------------------*/

void ledControl(int position, led_state s) {
	if (s == ON) {
		PTC->PSOR = MASK(position);
	} else {
		PTC->PCOR = MASK(position);
	}
}

void moveForward() {
	TPM0_C0V += (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C2V = 0;
	TPM0_C3V += (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C1V = 0;
	TPM0_C4V += (TPM0->MOD * RIGHTDUTYCYCLE);
	TPM0_C5V = 0;
	TPM1_C0V += (TPM1->MOD * RIGHTDUTYCYCLE);
	TPM1_C1V = 0;
}

void stopForward() {
	TPM0_C0V -= (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C3V -= (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C4V -= (TPM0->MOD * RIGHTDUTYCYCLE);
	TPM1_C0V -= (TPM1->MOD * RIGHTDUTYCYCLE);
	//TPM0_C0V = 0;
	//TPM0_C3V = 0;
	//TPM0_C4V = 0;
	//TPM1_C0V = 0;
}

void moveBackward() {
	TPM0_C0V = 0;
	TPM0_C2V = (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C3V = 0;
	TPM0_C1V = (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C4V = 0;
	TPM0_C5V = (TPM0->MOD * RIGHTDUTYCYCLE);
	TPM1_C0V = 0;
	TPM1_C1V = (TPM1->MOD * RIGHTDUTYCYCLE);
}

void stopBackward() {
	TPM0_C2V -= (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C1V -= (TPM0->MOD * LEFTDUTYCYCLE);
	TPM0_C5V -= (TPM0->MOD * RIGHTDUTYCYCLE);
	TPM1_C1V -= (TPM1->MOD * RIGHTDUTYCYCLE);
	//TPM0_C2V = 0;
	//TPM0_C1V = 0;
	//TPM0_C5V = 0;
	//TPM1_C1V = 0;
}

void moveLeft() {
	if (isBackward) {
		TPM0_C4V = 0;
		TPM1_C0V = 0;
		//if (isMoving) {
			//TPM0_C5V += (TPM0->MOD);
			//TPM1_C1V += (TPM1->MOD);
		//} else {
		TPM0_C5V += (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C1V += (TPM1->MOD * RIGHTDUTYCYCLE); // TODO: what happens if CnV > MOD?
		//}
	} else {
		TPM0_C5V = 0;
		TPM1_C1V = 0;	
		//if (isMoving) {
			//TPM0_C4V += (TPM0->MOD);
			//TPM1_C0V += (TPM1->MOD);
		//} else {
			TPM0_C4V += (TPM0->MOD * RIGHTDUTYCYCLE);
			TPM1_C0V += (TPM1->MOD * RIGHTDUTYCYCLE);
		//}
	}
}

void stopLeft() {
		if (isBackward) {
		TPM0_C5V -= (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C1V -= (TPM1->MOD * RIGHTDUTYCYCLE);
	} else {
		TPM0_C4V -= (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C0V -= (TPM1->MOD * RIGHTDUTYCYCLE);
	}
}

void moveRight () {
	if (isBackward) {
		TPM0_C0V = 0;
		TPM0_C2V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C3V = 0;
		TPM0_C1V += (TPM0->MOD * LEFTDUTYCYCLE);
	} else {
		TPM0_C0V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C2V = 0;
		TPM0_C3V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C1V = 0;
	}
}

void stopRight() {
	if (isBackward) {
		TPM0_C2V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C1V -= (TPM0->MOD * LEFTDUTYCYCLE);
	} else {
		TPM0_C0V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C3V -= (TPM0->MOD * LEFTDUTYCYCLE);
	}
}

/*----------------------------------------------------------------------------
 * TASKS
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
  for (;;) {
		osSemaphoreAcquire(dataIn, osWaitForever);
		if (START_MUSIC(rx_data)) {
			
		} else if (RUN_MUSIC(rx_data)) {
			
		} else if (END_MUSIC(rx_data)) {
			
		} else {
			if (MOVE_MASK(rx_data)) {
				isMoving = true;
			} else {
				isMoving = false;
			}
			if (DIRECTION_MASK(rx_data) == MOVE_BACKWARD) {
				isBackward = !isBackward;
			}
	
			osMessageQueuePut(motorMsg, &rx_data, NULL, 0); 
		} 
		//isMoving = !isMoving;
	}
}

void tMotorControl (void *argument) {
	uint8_t rx;
  for (;;) {
		osMessageQueueGet(motorMsg, &rx, NULL, osWaitForever);
		// TODO: input protection?
		if (MOVE_MASK(rx)) {
			if (DIRECTION_MASK(rx) == MOVE_FORWARD) {
				moveForward();
			} else if (DIRECTION_MASK(rx) == MOVE_BACKWARD) {
				moveBackward();
			} else if (DIRECTION_MASK(rx) == MOVE_LEFT) {
				moveLeft();
			} else if (DIRECTION_MASK(rx) == MOVE_RIGHT) {
				moveRight();
			}
		} else {
			if (DIRECTION_MASK(rx) == MOVE_FORWARD) {
				stopForward();
			} else if (DIRECTION_MASK(rx) == MOVE_BACKWARD) {
				stopBackward();
			} else if (DIRECTION_MASK(rx) == MOVE_LEFT) {
				stopLeft();
			} else if (DIRECTION_MASK(rx) == MOVE_RIGHT) {
				stopRight();
			}
		} 
		//moveForward();
	}
}

void tLEDGreen (void *argument) {
	int j = 0;
	for (;;) {
		while (isMoving) {
			for (int i = 0; i < 8; i++) {
				ledControl(green_pos[i], OFF);
			}
			j = (j + 1) % 8;
			ledControl(green_pos[j], ON);
			osDelay(100);
			ledControl(green_pos[j], OFF);
		} 
		while(!isMoving){
			for (int i = 0; i < 8; i++) {
				ledControl(green_pos[i], ON);
			}
		}
	}
}

void tLEDRed (void *argument) {
  for (;;) {
		while (isMoving) {
			ledControl(PTC_RED, ON);
			osDelay(500);
			ledControl(PTC_RED,OFF);
			osDelay(500);
		} 
		while (!isMoving) {
			ledControl(PTC_RED, ON);
			osDelay(250);
			ledControl(PTC_RED,OFF);
			osDelay(250);
		}
	}
}

void tAudio (void *argument) {
	int introLength = sizeof(intro)/sizeof(intro[0]);
  for (;;) {
		for (int i = 0; i < introLength; i++) {
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
		rx_data = UART2->D;
		osSemaphoreRelease(dataIn);
	}
}

/*----------------------------------------------------------------------------
 * MAIN FUNCTION
 *---------------------------------------------------------------------------*/

int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
	initGPIO();
	initMotors();
	initBuzzer();
	initUART2(BAUD_RATE);
	__enable_irq();
 
  osKernelInitialize();
	dataIn = osSemaphoreNew(1,0,NULL);
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
  osThreadNew(tBrain, NULL, &normal1);
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLEDRed, NULL, NULL);
	osThreadNew(tLEDGreen, NULL, NULL);
	//osThreadNew(tAudio, NULL, NULL);	
  osKernelStart();                      
  for (;;) {}
}
