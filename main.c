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
#define RIGHTDUTYCYCLE 0.85

/* synchronization related constants and variables */
#define MSG_COUNT 1

uint8_t rx_data = 0x00;

osSemaphoreId_t dataIn;
osEventFlagsId_t connectIn;

const osThreadAttr_t normal1 = {
	.priority = osPriorityNormal1
};

/* other variables and enums */
volatile bool isMoving = false;
volatile bool isForward = false;
volatile bool isBackward = false;
bool isIntroPlayed = false; // not volatile to let intro play to completion
volatile bool isEnd = false;

typedef enum {OFF, ON} led_state;

/* green led pins, PORTC */
static int green_pos[] = {7,0,3,4,5,6,10,11};

/* tune related data */
// MOD values for each note
static uint32_t notes[] = {
				0, // rest  0
        1430, // C4 - 262 Hz 1
        1274, // D4 - 294 Hz  2
        1135, // E4 - 330 Hz  3
        1073, // F4 - 349 Hz  4
				1012, // F4s - 370 Hz  5
        955, // G4 - 392 Hz  6
				851, // A4 - 440 Hz  7
				804, // A4s - 466 Hz  8
        758, // B4 - 494 Hz  9
				716, // C5 - 523 Hz  10
				676, // C5s - 554 Hz  11
				638, // D5 - 587 Hz  12
				568, // E5 - 659 Hz  13
				536, // F5 - 698Hz  14
				506, // F5s - 740Hz  15
				477, // G5 - 784Hz  16
				425, // A5 - 880 Hz  17
				378, // B5 - 988 Hz  18
				357, // C6 - 1046 Hz  19
				318, // D6 - 1175Hz  20
				1352, // C4s - 277Hz  21
				267  // F6 - 1397Hz  22
};

// Pokemon GSC Champion/Red Intro
static uint32_t intro[] = {
	3,0,3,0, 3,0,3,2, 3,3,0,3,3,0, 3,3,0,3,3,9,
	3,3,0,3,3,0, 3,3,0,3,3,9, 3,3,0,3,3,0, 3,3,0,3,3,10,
	3,3,9,3,3,9, 3,3,9,3,3,2, 3,3,9,3,3,9, 3,3,9,3,3,4,
	3,3,9,3,3,9, 3,3,9,3,3,2, 3,3,9,3,3,9, 3,3,10,3,3,11,
	13,3, 13,13,8,9,10,11 
};

static uint8_t introDuration[] = {
	2,2,2,2, 2,2,2,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2, 3,3,2,3,3,2,
	1,1, 1,2,4,4,4,4
};

// Pokemon GSC Champion/Red Theme (Loop)
static uint32_t battle[] = {
	13,9,13,12, 11,10,9,9, 9,13, 7,7,9,7,7,11,
	13,9,13,12, 11,10,9,9,10, 13,17, 13,18,
	13,13,0,13,0,13,0,13, 0,13,0,13,13,13, 13,13,0,13,0,13,0,13, 0,13,0,13,15,14,
	13,3, 11,21, 3, 
	4, 5, 13,12,11,10, 13,12,11,10,9,9, 
	13,12,11,10, 13,12,11,10,9,9, 13, 18,13,13,
	18, 17, 3,3,9,3,3,9, 3,3,9,3,3,7, 
	3,3,9,3,3,9, 3,3,9,3,3,12, 13,13, 15,13, 
	13,13, 15,17
};

static uint8_t battleDuration[] = {
	5,5,3,3, 2,2,2,2, 1,1, 3,3,2,3,3,2,
	5,5,3,3, 2,2,2,3,3, 1,1, 1,1, 
	3,3,3,3,3,3,3,3, 3,3,3,3,2,2, 3,3,3,3,3,3,3,3, 3,3,3,3,2,2,
	1,1, 1,1, 6,
	0, 0, 2,2,2,2, 3,3,3,3,2,2, 
	2,2,2,2, 3,3,3,3,2,2, 0, 1,2,2,
	0, 0, 3,3,2,3,3,2, 3,3,2,3,3,2,
	3,3,2,3,3,2, 3,3,2,3,3,2, 1,1, 1,1,
	1,1, 1,1
};

// Pokemon GSC Victory! (Gym Battle)
static uint32_t victory[] = {
	17,14,17,20,17,20, 22,14,17,20,
	12,10,9,0,10,9,7,0, 9,7,6,4,7,7, 12,10,9,0,10,9,7,0, 9,7,6,4,2,0,18,19,
	20,19,18,0,19,18,17,0, 18,17,16,14,17,17, 20,19,18,0,19,18,17,0, 18,17,16,14,12,0
};

static uint8_t victoryDuration[] = {
	5,4,4,5,4,4, 5,4,4,1,
	3,3,3,3,3,3,3,3, 3,3,3,3,2,2, 3,3,3,3,3,3,3,3, 3,3,3,3,2,3,4,4,
	3,3,3,3,3,3,3,3, 3,3,3,3,2,2, 3,3,3,3,3,3,3,3, 3,3,3,3,2,2
}; 

static float durationChoose[] = {4, 2, 1, 0.5, 0.25, 1.5, 8};

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
	
	TPM0->MOD = 100;  // using 30kHz
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
}

void moveLeft() {
	if (isBackward) {
		TPM0_C4V = 0;
		TPM1_C0V = 0;
		TPM0_C5V += (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C1V += (TPM1->MOD * RIGHTDUTYCYCLE);
		TPM0_C0V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C3V += (TPM0->MOD * LEFTDUTYCYCLE);
	} else {
		TPM0_C5V = 0;
		TPM1_C1V = 0;	
		TPM0_C4V += (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C0V += (TPM1->MOD * RIGHTDUTYCYCLE);
		TPM0_C2V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C1V += (TPM0->MOD * LEFTDUTYCYCLE);
	}
}

void stopLeft() {
	if (isBackward) {
		TPM0_C5V -= (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C1V -= (TPM1->MOD * RIGHTDUTYCYCLE);
		TPM0_C0V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C3V -= (TPM0->MOD * LEFTDUTYCYCLE);
	} else {
		TPM0_C4V -= (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C0V -= (TPM1->MOD * RIGHTDUTYCYCLE);
		TPM0_C2V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C1V -= (TPM0->MOD * LEFTDUTYCYCLE);
	}
}

void moveRight () {
	if (isBackward) {
		TPM0_C0V = 0;
		TPM0_C2V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C3V = 0;
		TPM0_C1V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C4V += (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C0V += (TPM1->MOD * RIGHTDUTYCYCLE);
	} else {
		TPM0_C0V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C2V = 0;
		TPM0_C3V += (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C1V = 0;
		TPM0_C5V += (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C1V += (TPM1->MOD * RIGHTDUTYCYCLE);
	}
}

void stopRight() {
	if (isBackward) {
		TPM0_C2V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C1V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C4V -= (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C0V -= (TPM1->MOD * RIGHTDUTYCYCLE);
	} else {
		TPM0_C0V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C3V -= (TPM0->MOD * LEFTDUTYCYCLE);
		TPM0_C5V -= (TPM0->MOD * RIGHTDUTYCYCLE);
		TPM1_C1V -= (TPM1->MOD * RIGHTDUTYCYCLE);
	}
}

/*----------------------------------------------------------------------------
 * TASKS
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
  for (;;) {
		osSemaphoreAcquire(dataIn, osWaitForever);
		
		if (START_MUSIC(rx_data)) {
			osEventFlagsSet(connectIn, 0x00000003);
			isIntroPlayed = false;
			isEnd = false;
		} else if (RUN_MUSIC(rx_data)) {
			isIntroPlayed = true; // this is for debugging
			isEnd = false;
		} else if (END_MUSIC(rx_data)) {
			isEnd = true;
		} else {
			
			if (DIRECTION_MASK(rx_data) == MOVE_FORWARD) {
				isForward = !isForward;
			}
			if (DIRECTION_MASK(rx_data) == MOVE_BACKWARD) {
				isBackward = !isBackward;
			}
			if (MOVE_MASK(rx_data) || isForward || isBackward) {
				isMoving = true;
			} else {
				isMoving = false;
			}
	
			osEventFlagsSet(connectIn, 0x00000004);
		} 
	}
}

void tMotorControl (void *argument) {
  for (;;) {
		osEventFlagsWait(connectIn, 0x00000004, osFlagsWaitAll, osWaitForever);
		
		if (MOVE_MASK(rx_data)) {
			if (DIRECTION_MASK(rx_data) == MOVE_FORWARD) {
				moveForward();
			} else if (DIRECTION_MASK(rx_data) == MOVE_BACKWARD) {
				moveBackward();
			} else if (DIRECTION_MASK(rx_data) == MOVE_LEFT) {
				moveLeft();
			} else if (DIRECTION_MASK(rx_data) == MOVE_RIGHT) {
				moveRight();
			}
		} else {
			if (DIRECTION_MASK(rx_data) == MOVE_FORWARD) {
				stopForward();
			} else if (DIRECTION_MASK(rx_data) == MOVE_BACKWARD) {
				stopBackward();
			} else if (DIRECTION_MASK(rx_data) == MOVE_LEFT) {
				stopLeft();
			} else if (DIRECTION_MASK(rx_data) == MOVE_RIGHT) {
				stopRight();
			}
		} 
	}
}

void tLEDGreen (void *argument) {
	int j = 0;
	bool flash = false;
	for (int i = 0; i < 8; i++) {
		ledControl(green_pos[i], ON);
	}
	osEventFlagsWait(connectIn, 0x00000001, osFlagsWaitAny, osWaitForever);
	// connection established LED flashes
	for (int blinkCount = 0; blinkCount < 2; blinkCount++) {
		for (int i = 0; i < 8; i++) {
			ledControl(green_pos[i], OFF);
		}
		osDelay(500);
		for (int i = 0; i < 8; i++) {
			ledControl(green_pos[i], ON);
		}
		osDelay(500);
	}
	
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
	int battleLength = sizeof(battle)/sizeof(battle[0]);
	int victoryLength = sizeof(victory)/sizeof(victory[0]);
	int bCounter = 0;
	int vCounter = 0;
	osEventFlagsWait(connectIn, 0x00000002, osFlagsWaitAny, osWaitForever);
  for (;;) {
		if (!isIntroPlayed){
			for (int i = 0; i < introLength; i++) {
				TPM2->MOD = notes[intro[i]];
				TPM2_C0V = notes[intro[i]] / 2;
				osDelay(QUARTERBEAT*durationChoose[introDuration[i]]);
				TPM2->MOD = 0;
				TPM2_C0V = 0;
				osDelay(NOTEDELAY);
			}
			isIntroPlayed = true;
			bCounter = 0;
			vCounter = 0;
		} else if (!isEnd) {
			vCounter = 0;
			TPM2->MOD = notes[battle[bCounter]];
			TPM2_C0V = notes[battle[bCounter]] / 2;
			osDelay(QUARTERBEAT*durationChoose[battleDuration[bCounter]]);
			TPM2->MOD = 0;
			TPM2_C0V = 0;
			osDelay(NOTEDELAY);
			bCounter = (bCounter + 1) % battleLength;
		}	else {
			bCounter = 0;
			TPM2->MOD = notes[victory[vCounter]];
			TPM2_C0V = notes[victory[vCounter]] / 2;
			osDelay(QUARTERBEAT*durationChoose[victoryDuration[vCounter]]);
			TPM2->MOD = 0;
			TPM2_C0V = 0;
			osDelay(NOTEDELAY);
			vCounter = (vCounter + 1) % victoryLength;
		}
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
	connectIn = osEventFlagsNew(NULL);
  osThreadNew(tBrain, NULL, &normal1);
	osThreadNew(tMotorControl, NULL, &normal1);
	osThreadNew(tLEDRed, NULL, NULL);
	osThreadNew(tLEDGreen, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);	
  osKernelStart();                      
  for (;;) {}
}
