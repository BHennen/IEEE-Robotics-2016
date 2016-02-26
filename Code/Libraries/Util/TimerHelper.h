#ifndef TimerHelper_H
#define TimerHelper_H

#include "TimerProgMems.h"

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

	enum MODES : byte
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
	//convert MODES into 8bit modes
	const byte MODES8[]
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
		T0_FALLING,
		T0_RISING,
		PRESCALE_32,
		PRESCALE_128
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

	enum PORTS : byte
	{
		NO_PORT = 0,

		TOGGLE_A_ON_COMPARE = bit(COMnA0),
		CLEAR_A_ON_COMPARE = bit(COMnA1),
		SET_A_ON_COMPARE = bit(COMnA0) | bit(COMnA1),

		TOGGLE_B_ON_COMPARE = bit(COMnB0),
		CLEAR_B_ON_COMPARE = bit(COMnB1),
		SET_B_ON_COMPARE = bit(COMnB0) | bit(COMnB1),

		TOGGLE_C_ON_COMPARE = bit(COMnC0),
		CLEAR_C_ON_COMPARE = bit(COMnC1),
		SET_C_ON_COMPARE = bit(COMnC0) | bit(COMnC1),
	};

	inline bool SetMode(const byte pin_number, const MODES mode, const CLOCK clock, const PORTS ports)
	{
		if(mode < 0 || mode > 15 || clock < 0 || clock > 9)  // sanity checks
			return false;

		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return false;

		//Select a mode depending on 8 bit or 16 bit timer
		byte selected_mode = testTimer8bit(timer) ? MODES8[mode] : mode;
		if(mode == RESERVED) return false; //Do nothing for a reserved mode

		//Make sure clock is valid
		byte clock_number;
		if((timer == TIMER2) || (timer == TIMER2A) || (timer == TIMER2B))
		{
			clock_number = CLOCK_to_timer2[clock];
		}
		if(clock_number > 7) return false; // We didn't pick a valid option for the timer we have

		//TODO: Make sure requested ports are valid

		//Get correct ports to write to.
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
		noInterrupts();
		*TCCRnA = 0;
		*TCCRnB = 0;

		*TCCRnA |= (Modes[mode][0]) | ports;
		*TCCRnB |= (Modes[mode][1]) | clock;
		interrupts();

		return true;
	}

	inline bool SetOCR(const byte pin_number, const uint16_t value)
	{
		uint8_t timer = digitalPinToTimer(pin_number);
		if(timer == NOT_ON_TIMER) return false;

		volatile uint8_t* OCRnX = timerToOCRnX(timer);
		if(OCRnX == NOT_ON_TIMER) return false;

		if(testTimer8bit(timer))
		{
			uint8_t new_val = static_cast<uint8_t>(value);
			noInterrupts();
			*OCRnX = new_val;
			interrupts();
		}
		else
		{
			uint16_t new_val = static_cast<uint16_t>(value);
			noInterrupts();
			*OCRnX = new_val;
			interrupts();
		}
		return true;		
	}
}

#endif