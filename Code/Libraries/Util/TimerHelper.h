#ifndef TimerHelper_H
#define TimerHelper_H

#include "TimerProgMems.h"
#include <util\atomic.h>

#define roundDivide(n,d)	((((n) < 0) ^ ((d) < 0)) ? (((n) - (d)/2)/(d)) : (((n) + (d)/2)/(d)))
#define ceilDivide(n,d)		((((n) - 1)/(d)) + 1)

namespace Timer
{
	// TCCRnA, TCCRnB
	const byte Modes[16][2] =
	{
		{0,							0},							// 0: Normal, top = 0xFFFF
		{bit(WGMn0),				0},							// 1: PWM, Phase-correct, 8 bit, top = 0xFF
		{bit(WGMn1),				0},							// 2: PWM, Phase-correct, 9 bit, top = 0x1FF
		{bit(WGMn0) | bit(WGMn1),	0},							// 3: PWM, Phase-correct, 10 bit, top = 0x3FF
		{0,							bit(WGMn2)},				// 4: CTC, top = OCRnA
		{bit(WGMn0),				bit(WGMn2)},				// 5: Fast PWM, 8 bit, top = 0xFF
		{bit(WGMn1),				bit(WGMn2)},				// 6: Fast PWM, 9 bit, top = 0x1FF
		{bit(WGMn0) | bit(WGMn1),	bit(WGMn2)},				// 7: Fast PWM, 10 bit, top = 0x3FF
		{0,							bit(WGMn3)},				// 8: PWM, phase and frequency correct, top = ICRn    
		{bit(WGMn0),				bit(WGMn3)},				// 9: PWM, phase and frequency correct, top = OCRnA    
		{bit(WGMn1),				bit(WGMn3)},				// 10: PWM, phase correct, top = ICRnA    
		{bit(WGMn0) | bit(WGMn1),	bit(WGMn3)},				// 11: PWM, phase correct, top = OCRnA
		{0,							bit(WGMn2) | bit(WGMn3)},	// 12: CTC, top = ICRn    
		{bit(WGMn0),				bit(WGMn2) | bit(WGMn3)},	// 13: reserved
		{bit(WGMn1),				bit(WGMn2) | bit(WGMn3)},	// 14: Fast PWM, TOP = ICRn
		{bit(WGMn0) | bit(WGMn1),	bit(WGMn2) | bit(WGMn3)},	// 15: Fast PWM, TOP = OCRnA
	};
	//8bit timer modes
	const byte Modes8[8][2] =
	{
		{0,							0},				// 0: Normal, top = 0xFFFF
		{bit(WGM00),				0},				// 1: PWM, Phase-correct, 8 bit, top = 0xFF
		{bit(WGM01),				0},				// 2: CTC, top = OCRnA
		{bit(WGM00) | bit(WGM01),	0},				// 3: Fast PWM, 8 bit, top = 0xFF
		{0,							bit(WGM02)},	// 4: reserved
		{bit(WGM00),				bit(WGM02)},	// 5: PWM, phase correct, top = OCRnA
		{bit(WGM01),				bit(WGM02)},	// 6: reserved
		{bit(WGM00) | bit(WGM01),	bit(WGM02)},	// 7: Fast PWM, TOP = OCRnA
	};

	enum MODE : byte
	{
		NORMAL,				// 0: Normal, top = 0xFFFF
		PWM_PC_8B,			// 1: PWM, Phase-correct, 8 bit, top = 0xFF
		PWM_PC_9B,			// 2: PWM, Phase-correct, 9 bit, top = 0x1FF
		PWM_PC_10B,			// 3: PWM, Phase-correct, 10 bit, top = 0x3FF
		CTC_OCRnA,			// 4: CTC, top = OCRnA
		FAST_PWM_8b,		// 5: Fast PWM, 8 bit, top = 0xFF
		FAST_PWM_9b,		// 6: Fast PWM, 9 bit, top = 0x1FF
		FAST_PWM_10b,		// 7: Fast PWM, 10 bit, top = 0x3FF
		PWM_PC_FC_ICRn,		// 8: PWM, phase and frequency correct, top = ICRn    
		PWM_PC_FC_OCRnA,	// 9: PWM, phase and frequency correct, top = OCRnA    
		PWM_PC_ICRn,		// 10: PWM, phase correct, top = ICRnA    
		PWM_PC_OCRnA,		// 11: PWM, phase correct, top = OCRnA
		CTC_ICRn,			// 12: CTC, top = ICRn    
		RESERVED,			// 13: reserved
		FAST_PWM_ICRn,		// 14: Fast PWM, TOP = ICRn
		FAST_PWM_OCRnA		// 15: Fast PWM, TOP = OCRnA
	};
	//convert MODE into 8bit modes
	const byte MODE8[]
	{
		0,	// 0: Normal, top = 0xFFFF
		1,	// 1: PWM, Phase-correct, 8 bit, top = 0xFF
		13,	// not used
		13,	// not used
		2,	// 2: CTC, top = OCRnA
		3,	// 3: Fast PWM, 8 bit, top = 0xFF
		13,	// not used
		13,	// not used
		13,	// not used
		13,	// not used
		13,	// not used
		5,	// 5: PWM, phase correct, top = OCRnA
		13,	// not used
		13,	// not used
		13,	// not used
		7	// 7: Fast PWM, TOP = OCRnA
	};

	enum CLOCK : byte
	{
		NO_CLOCK, 
		PRESCALE_1, 
		PRESCALE_8,
		PRESCALE_64,
		PRESCALE_256,
		PRESCALE_1024,
		Tn_FALLING,
		Tn_RISING,
		PRESCALE_32,
		PRESCALE_128,
		INVALID_CLOCK,
		AUTO_SCALE
	};
	//conversion from CLOCK to timer2
	const byte CLOCK_to_timer2[]
	{
		0,
		1,
		2,
		4,
		6,
		7,
		8,
		9,
		3,
		5
	};

	enum PORT_OUTPUT_MODE : byte
	{
		NO_OUTPUT = 0,

		TOGGLE_ON_COMPARE,
		CLEAR_ON_COMPARE,
		SET_ON_COMPARE,
	};

	//Starts a timer on
	inline bool SetMode(const byte pin_number, const MODE mode, const CLOCK clock, const PORT_OUTPUT_MODE port_output_mode)
	{
		if(mode < 0 || mode > 15 || clock < 0 || clock > 9)  // sanity checks
			return false;

		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return false;

		byte selected_mode = mode;
		//Get mode based on whether or not the timer is 8 or 16 bit
		if(testTimer8bit(timer))
		{
			selected_mode = MODE8[mode];
		}
		if(mode == RESERVED) return false; //Do nothing for a reserved mode

		//Make sure clock is valid
		byte clock_number;
		if((timer == TIMER2) || (timer == TIMER2A) || (timer == TIMER2B))
		{
			clock_number = CLOCK_to_timer2[clock];
		}
		if(clock_number > 7) return false; // We didn't pick a valid option for the timer we have

		//Get correct ports to write to.
		uint8_t selected_port_mode = 0;
		if(port_output_mode == TOGGLE_ON_COMPARE)
		{
			selected_port_mode = bit(timerToCOMnX0(timer));
		}
		else if(port_output_mode == CLEAR_ON_COMPARE)
		{
			selected_port_mode = bit(timerToCOMnX1(timer));
		}
		else if(port_output_mode == SET_ON_COMPARE)
		{
			selected_port_mode = bit(timerToCOMnX0(timer)) | bit(timerToCOMnX1(timer));
		}

		//Get addresses
		volatile uint8_t* TCCRnA = timerToTCCRnA(pin_number);
		volatile uint8_t* TCCRnB = timerToTCCRnB(pin_number);
		if(TCCRnA == NOT_ON_TIMER)
		{
			return false;
		}
		else if(TCCRnB == NOT_ON_TIMER)
		{
			return false;
		}

		// reset existing flags
		*TCCRnA = 0;
		*TCCRnB = 0;

		*TCCRnA |= (Modes[mode][0]) | selected_port_mode;
		*TCCRnB |= (Modes[mode][1]) | clock;

		return true;
	}

	//Sets the OCRnX of the selected pin. Return true if successful.
	inline bool SetOCR(const byte pin_number, const uint16_t value)
	{
		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return false;

		volatile uint8_t* OCRnX = timerToOCRnX(timer);
		if(OCRnX == NOT_ON_TIMER) return false;

		if(testTimer8bit(timer))
		{
			*OCRnX = static_cast<uint8_t>(value);; //writing to 8 bit doesn't require atomicity
		}
		else
		{
			//Make sure the write is an atomic operation.
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				*OCRnX = value;
			}
		}
		return true;		
	}

	//Enables the output compare interrupt for the given pin. Return true if successful.
	inline bool EnableInterrupt(const byte pin_number)
	{
		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return false;

		volatile uint8_t* TIFRn = timerToTIFRn(timer);
		uint8_t OCFnX = timerToOCFnX(timer);
		volatile uint8_t* TIMSKn = timerToTIMSKn(timer);
		uint8_t OCIEnX = timerToOCIEnX(timer);
		if((TIFRn == NOT_ON_TIMER) || (OCFnX == NOT_ON_TIMER) || (TIMSKn == NOT_ON_TIMER) || (OCIEnX == NOT_ON_TIMER))
			return false; //invalid pin for interrupt.

		*TIFRn |= bit(OCFnX);  // clear any pending interrupts on the pin
		*TIMSKn |= bit(OCIEnX); // enable the output compare interrupt
		return true;
	}

	//Disables the output compare interrupt for the given pin. Return true if successful.
	inline bool DisableInterrupt(const byte pin_number)
	{
		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return false;

		volatile uint8_t* TIMSKn = timerToTIMSKn(timer);
		uint8_t OCIEnX = timerToOCIEnX(timer);
		if((TIMSKn == NOT_ON_TIMER) || (OCIEnX == NOT_ON_TIMER))
			return false; //invalid pin for interrupt.

		*TIMSKn &= ~bit(OCIEnX); // disable the output compare interrupt
		return true;
	}

	//Determines the necessary prescale to generate a given frequency on a given pin.
	inline CLOCK GetPrescaleForFrequency(const unsigned long frequency, const byte pin_number)
	{
		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return INVALID_CLOCK;

		unsigned int prescale;

		//test if timer is 8 bit & set prescale. Calculated by using the maximum possible OCR + 1. Also round instead of truncate.
		//Formula: Since F_CPU/prescale/frequency - 1 = OCR, then if we assume OCR to be maximum value (either 255 or 65535)
		//		   then F_CPU/(MAX_OCR + 1)/frequency >= prescale
		if(testTimer8bit(timer))
		{
			prescale = ceilDivide(F_CPU / 256, frequency);
		}
		else
		{
			prescale = ceilDivide(ceilDivide(F_CPU, 65536), frequency);
		}

		//Set prescale.
		if(prescale <= 1)
		{
			return PRESCALE_1;
		}
		else if(prescale <= 8)
		{
			return PRESCALE_8;
		}
		else if((timer == TIMER2) || (timer == TIMER2A) || (timer == TIMER2B) && prescale <= 32)
		{
			return PRESCALE_32;
		}		
		else if(prescale <= 64)
		{
			return PRESCALE_64;
		}
		else if((timer == TIMER2) || (timer == TIMER2A) || (timer == TIMER2B) && prescale <= 128)
		{
			return PRESCALE_128;
		}
		else if(prescale <= 256)
		{
			return PRESCALE_256;
		}
		else if(prescale <= 1024)
		{
			return PRESCALE_1024;
		}
		else
		{
			return INVALID_CLOCK; //Frequency too small
		}
	}

	//Determine the OCR value to generate a given frequency for the given pin and given prescale.
	//Automatically sets lowest possible prescale if not given as an argument.
	//Return -1 if error
	inline long GetOCR(const unsigned long frequency, const byte pin_number, const CLOCK prescale = AUTO_SCALE)
	{
		//Check if this pin supports a timer
		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return -1;

		//Check selected prescale & generate it if not selected. If prescale not available, return
		CLOCK selected_prescale = (prescale == AUTO_SCALE) ? GetPrescaleForFrequency(frequency, pin_number) : prescale;
		if((selected_prescale == INVALID_CLOCK) || (selected_prescale == NO_CLOCK) || (selected_prescale == Tn_RISING) || (selected_prescale == Tn_FALLING)) return -1;

		//Convert prescale into actual number
		uint16_t prescale_number;
		switch(selected_prescale)
		{
			case PRESCALE_1:
				prescale_number = 1;
				break;
			case PRESCALE_8:
				prescale_number = 8;
				break;
			case PRESCALE_32:
				prescale_number = 32;
				break;
			case PRESCALE_64:
				prescale_number = 64;
				break;
			case PRESCALE_128:
				prescale_number = 128;
				break;
			case PRESCALE_256:
				prescale_number = 256;
				break;
			case PRESCALE_1024:
				prescale_number = 1024;
				break;
		}

		//Generate OCR
		long OCR = roundDivide(roundDivide(F_CPU, prescale_number), frequency) - 1;

		//Check if OCR reasonable
		if(testTimer8bit(timer))
		{
			if(OCR >= 256 || OCR < 0) return -1;
		}
		else if(OCR >= 65536 || OCR < 0) return -1;

		return OCR;
	}
}

#endif