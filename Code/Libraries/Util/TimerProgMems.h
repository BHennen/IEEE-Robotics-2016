#ifndef TimerProgMems_H
#define TimerProgMems_H
#include <Arduino.h>

#define timerToTCCRnA(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCCRnA_PGM + (P))) )
#define timerToTCCRnB(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCCRnB_PGM + (P))) )
#define timerToTCCRnC(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCCRnC_PGM + (P))) )

#define timerToTCNTn(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_TCNTn_PGM + (P))) )

#define timerToOCRnX(P) ( (volatile uint8_t *)( pgm_read_word( timer_to_OCRnX_PGM + (P))) )

#define testTimer8bit(P) ( (P == TIMER0A) || (P == TIMER0B) || (P == TIMER2) || (P == TIMER2A) || (P == TIMER2B) )

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

//TCCRnA
#define COMnA1  7
#define COMnA0  6
#define COMnB1  5
#define COMnB0  4
#define COMnC1  3
#define COMnC0  2
#define WGMn1   1
#define WGMn0   0

//TCCRnB
#define WGMn3   4
#define WGMn2   3

#endif