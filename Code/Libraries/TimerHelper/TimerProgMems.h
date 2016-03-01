#ifndef TimerProgMems_H
#define TimerProgMems_H
#include <Arduino.h>

#define NUM_TIMER_INTERRUPTS 16
#define NUM_8_BIT_TIMERS 4
#define NUM_16_BIT_TIMERS (NUM_TIMER_INTERRUPTS - NUM_8_BIT_TIMERS)

#define timerToTCCRnA(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCCRnA_PGM + (P))) )
#define timerToTCCRnB(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCCRnB_PGM + (P))) )
#define timerToTCCRnC(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCCRnC_PGM + (P))) )

#define timerToTCNTn(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCNTn_PGM + (P))) )

#define timerToOCRnX(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_OCRnX_PGM + (P))) )

#define testTimer8bit(P) ( (P == TIMER0A) || (P == TIMER0B) || (P == TIMER2) || (P == TIMER2A) || (P == TIMER2B) )

#define timerToTIFRn(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TIFRn_PGM + (P))) )
#define timerToOCFnX(P) ( (uint8_t)( pgm_read_word( timer_to_OCFnX_PGM + (P))) )
#define timerToTIMSKn(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TIMSKn_PGM + (P))) )
#define timerToOCIEnX(P) ( (uint8_t)( pgm_read_word( timer_to_OCIEnX_PGM + (P))) )
#define timerToCOMnX1(P) ( (uint8_t)( pgm_read_word( timer_to_COMnX1_PGM + (P))) )
#define timerToCOMnX0(P) ( (uint8_t)( pgm_read_word( timer_to_COMnX0_PGM + (P))) )
#define timerToInterrupt(P) ( (uint8_t)( pgm_read_word( timer_to_interrupt_PGM + (P))) )
#define timerToTimerNumber(P) ( (int8_t)( pgm_read_word( timer_to_timer_number_PGM + (P))) )

//Timer to TCCRnA
const uint16_t PROGMEM timer_to_TCCRnA_PGM[] = {
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) &TCCR0A,
	(uint16_t) &TCCR0A,
	(uint16_t) &TCCR1A,
	(uint16_t) &TCCR1A,
	(uint16_t) &TCCR1A,
	(uint16_t) &TCCR2A,
	(uint16_t) &TCCR2A,
	(uint16_t) &TCCR2A,
	(uint16_t) &TCCR3A,
	(uint16_t) &TCCR3A,
	(uint16_t) &TCCR3A,
	(uint16_t) &TCCR4A,
	(uint16_t) &TCCR4A,
	(uint16_t) &TCCR4A,
	(uint16_t) &TCCR4A,
	(uint16_t) &TCCR5A,
	(uint16_t) &TCCR5A,
	(uint16_t) &TCCR5A
};

//Timer to TCCRnB
const uint16_t PROGMEM timer_to_TCCRnB_PGM[] = {
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) &TCCR0B,
	(uint16_t) &TCCR0B,
	(uint16_t) &TCCR1B,
	(uint16_t) &TCCR1B,
	(uint16_t) &TCCR1B,
	(uint16_t) &TCCR2B,
	(uint16_t) &TCCR2B,
	(uint16_t) &TCCR2B,
	(uint16_t) &TCCR3B,
	(uint16_t) &TCCR3B,
	(uint16_t) &TCCR3B,
	(uint16_t) &TCCR4B,
	(uint16_t) &TCCR4B,
	(uint16_t) &TCCR4B,
	(uint16_t) &TCCR4B,
	(uint16_t) &TCCR5B,
	(uint16_t) &TCCR5B,
	(uint16_t) &TCCR5B
};

//Timer to TCCRnC
const uint16_t PROGMEM timer_to_TCCRnC_PGM[] = {
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) &TCCR1C,
	(uint16_t) &TCCR1C,
	(uint16_t) &TCCR1C,
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) NOT_ON_TIMER,
	(uint16_t) &TCCR3C,
	(uint16_t) &TCCR3C,
	(uint16_t) &TCCR3C,
	(uint16_t) &TCCR4C,
	(uint16_t) &TCCR4C,
	(uint16_t) &TCCR4C,
	(uint16_t) &TCCR4C,
	(uint16_t) &TCCR5C,
	(uint16_t) &TCCR5C,
	(uint16_t) &TCCR5C
};

//Timer to TCNTn
const uint16_t PROGMEM timer_to_TCNTn_PGM[] = {
	(uint16_t)NOT_ON_TIMER,
	(uint16_t)&TCNT0,
	(uint16_t)&TCNT0,
	(uint16_t)&TCNT1,
	(uint16_t)&TCNT1,
	(uint16_t)&TCNT1,
	(uint16_t)&TCNT2,
	(uint16_t)&TCNT2,
	(uint16_t)&TCNT2,
	(uint16_t)&TCNT3,
	(uint16_t)&TCNT3,
	(uint16_t)&TCNT3,
	(uint16_t)&TCNT4,
	(uint16_t)&TCNT4,
	(uint16_t)&TCNT4,
	(uint16_t)&TCNT4,
	(uint16_t)&TCNT5,
	(uint16_t)&TCNT5,
	(uint16_t)&TCNT5
};

//Timer to OCRnX
const uint16_t PROGMEM timer_to_OCRnX_PGM[] = {
	(uint16_t)NOT_ON_TIMER,
	(uint16_t)&OCR0A,
	(uint16_t)&OCR0B,
	(uint16_t)&OCR1A,
	(uint16_t)&OCR1B,
	(uint16_t)&OCR1C,
	(uint16_t)NOT_ON_TIMER,
	(uint16_t)&OCR2A,
	(uint16_t)&OCR2B,
	(uint16_t)&OCR3A,
	(uint16_t)&OCR3B,
	(uint16_t)&OCR3C,
	(uint16_t)&OCR4A,
	(uint16_t)&OCR4B,
	(uint16_t)&OCR4C,
	(uint16_t)NOT_ON_TIMER,
	(uint16_t)&OCR5A,
	(uint16_t)&OCR5B,
	(uint16_t)&OCR5C
};

//Timer to TIFRn
const uint16_t PROGMEM timer_to_TIFRn_PGM[] = {
	(uint16_t)NOT_ON_TIMER,
	(uint16_t)&TIFR0,
	(uint16_t)&TIFR0,
	(uint16_t)&TIFR1,
	(uint16_t)&TIFR1,
	(uint16_t)&TIFR1,
	(uint16_t)&TIFR2,
	(uint16_t)&TIFR2,
	(uint16_t)&TIFR2,
	(uint16_t)&TIFR3,
	(uint16_t)&TIFR3,
	(uint16_t)&TIFR3,
	(uint16_t)&TIFR4,
	(uint16_t)&TIFR4,
	(uint16_t)&TIFR4,
	(uint16_t)&TIFR4,
	(uint16_t)&TIFR5,
	(uint16_t)&TIFR5,
	(uint16_t)&TIFR5
};

//Timer to OCFnX
const uint8_t PROGMEM timer_to_OCFnX_PGM[] = {
	NOT_ON_TIMER,
	OCF0A,
	OCF0B,
	OCF1A,
	OCF1B,
	OCF1C,
	NOT_ON_TIMER,
	OCF2A,
	OCF2B,
	OCF3A,
	OCF3B,
	OCF3C,
	OCF4A,
	OCF4B,
	OCF4C,
	NOT_ON_TIMER,
	OCF5A,
	OCF5B,
	OCF5C
};

//Timer to TIMSKn
const uint16_t PROGMEM timer_to_TIMSKn_PGM[] = {
	(uint16_t)NOT_ON_TIMER,
	(uint16_t)&TIMSK0,
	(uint16_t)&TIMSK0,
	(uint16_t)&TIMSK1,
	(uint16_t)&TIMSK1,
	(uint16_t)&TIMSK1,
	(uint16_t)&TIMSK2,
	(uint16_t)&TIMSK2,
	(uint16_t)&TIMSK2,
	(uint16_t)&TIMSK3,
	(uint16_t)&TIMSK3,
	(uint16_t)&TIMSK3,
	(uint16_t)&TIMSK4,
	(uint16_t)&TIMSK4,
	(uint16_t)&TIMSK4,
	(uint16_t)&TIMSK4,
	(uint16_t)&TIMSK5,
	(uint16_t)&TIMSK5,
	(uint16_t)&TIMSK5
};

//Timer to OCIEnX
const uint8_t PROGMEM timer_to_OCIEnX_PGM[] = {
	NOT_ON_TIMER,
	OCIE0A,
	OCIE0B,
	OCIE1A,
	OCIE1B,
	OCIE1C,
	NOT_ON_TIMER,
	OCIE2A,
	OCIE2B,
	OCIE3A,
	OCIE3B,
	OCIE3C,
	OCIE4A,
	OCIE4B,
	OCIE4C,
	NOT_ON_TIMER,
	OCIE5A,
	OCIE5B,
	OCIE5C
};

//Timer to COMnX1
const uint8_t PROGMEM timer_to_COMnX1_PGM[] = {
	NOT_ON_TIMER,
	COM0A1,
	COM0B1,
	COM1A1,
	COM1B1,
	COM1C1,
	NOT_ON_TIMER,
	COM2A1,
	COM2B1,
	COM3A1,
	COM3B1,
	COM3C1,
	COM4A1,
	COM4B1,
	COM4C1,
	NOT_ON_TIMER,
	COM5A1,
	COM5B1,
	COM5C1
};

//Timer to COMnX0
const uint8_t PROGMEM timer_to_COMnX0_PGM[] = {
	NOT_ON_TIMER,
	COM0A0,
	COM0B0,
	COM1A0,
	COM1B0,
	COM1C0,
	NOT_ON_TIMER,
	COM2A0,
	COM2B0,
	COM3A0,
	COM3B0,
	COM3C0,
	COM4A0,
	COM4B0,
	COM4C0,
	NOT_ON_TIMER,
	COM5A0,
	COM5B0,
	COM5C0
};

//Covert timer to timer number
const int8_t PROGMEM timer_to_timer_number_PGM[] = {
	-1,
	0,
	1,
	2,
	3,
	4,
	-1,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	-1,
	13,
	14,
	15
};

//Timer to interrupt
const uint8_t PROGMEM timer_to_interrupt_PGM[] = {
	99,				 //NOT_ON_TIMER
	0,				 //TIMER0A 
	1,				 //TIMER0B 
	2,				 //TIMER1A 
	3,				 //TIMER1B 
	4,				 //TIMER1C 
	99,				 //TIMER2  
	5,				 //TIMER2A 
	6,				 //TIMER2B 
	7,				 //TIMER3A 
	8,				 //TIMER3B 
	9,				 //TIMER3C 
	10,				 //TIMER4A 
	11,				 //TIMER4B 
	12,				 //TIMER4C 
	99,				 //TIMER4D 
	13,				 //TIMER5A 
	14,				 //TIMER5B 
	15				 //TIMER5C 
};

//TCCRnA
#define WGMn1   1
#define WGMn0   0

//TCCRnB
#define WGMn3   4
#define WGMn2   3

#endif