
#include "TimerHelper.h"

static volatile IntCallbackPtr timerIntFunc[NUM_TIMER_INTERRUPTS];

//Enables the output compare interrupt for the given pin. Return true if successful.
bool Timer::EnableInterrupt(const byte pin_number)
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

//Attaches and enables a timer interrupt to the given pin number.
bool Timer::AttachInterrupt(const byte pin_number, IntCallbackPtr userFunc, bool enable)
{
	uint8_t timer = digitalPinToTimer(pin_number);
	if(timer == NOT_ON_TIMER) return false;

	uint8_t timerInterruptNum = timerToInterrupt(timer);
	if(timerInterruptNum >= NUM_TIMER_INTERRUPTS) return false;

	timerIntFunc[timerInterruptNum] = userFunc;

	if(enable) EnableInterrupt(pin_number);
	return true;
}

//Disables the output compare interrupt for the given pin. Return true if successful.
bool Timer::DisableInterrupt(const byte pin_number)
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

ISR(TIMER0_COMPA_vect)
{
	if(timerIntFunc[0])
		timerIntFunc[0]->execute();
}

ISR(TIMER0_COMPB_vect)
{
	if(timerIntFunc[1])
		timerIntFunc[1]->execute();
}

ISR(TIMER1_COMPA_vect)
{
	if(timerIntFunc[2])
		timerIntFunc[2]->execute();
}

ISR(TIMER1_COMPB_vect)
{
	if(timerIntFunc[3])
		timerIntFunc[3]->execute();
}

ISR(TIMER1_COMPC_vect)
{
	if(timerIntFunc[4])
		timerIntFunc[4]->execute();
}

ISR(TIMER2_COMPA_vect)
{
	if(timerIntFunc[5])
		timerIntFunc[5]->execute();
}

ISR(TIMER2_COMPB_vect)
{
	if(timerIntFunc[6])
		timerIntFunc[6]->execute();
}

ISR(TIMER3_COMPA_vect)
{
	if(timerIntFunc[7])
		timerIntFunc[7]->execute();
}

ISR(TIMER3_COMPB_vect)
{
	if(timerIntFunc[8])
		timerIntFunc[8]->execute();
}

ISR(TIMER3_COMPC_vect)
{
	if(timerIntFunc[9])
		timerIntFunc[9]->execute();
}

ISR(TIMER4_COMPA_vect)
{
	if(timerIntFunc[10])
		timerIntFunc[10]->execute();
}

ISR(TIMER4_COMPB_vect)
{
	if(timerIntFunc[11])
		timerIntFunc[11]->execute();
}

ISR(TIMER4_COMPC_vect)
{
	if(timerIntFunc[12])
		timerIntFunc[12]->execute();
}

ISR(TIMER5_COMPA_vect)
{
	if(timerIntFunc[13])
		timerIntFunc[13]->execute();
}

ISR(TIMER5_COMPB_vect)
{
	if(timerIntFunc[14])
		timerIntFunc[14]->execute();
}

ISR(TIMER5_COMPC_vect)
{
	if(timerIntFunc[15])
		timerIntFunc[15]->execute();
}