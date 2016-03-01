#ifndef TimerTone_h
#define TimerTone_h

#include "TimerHelper.h"

class TimerTone
{
public:
	//Create a new TimerTone instance which can generate a single 
	//tone at a time using the specified timer and output pins.
	TimerTone(byte timer_pin, byte output_pin);

	// Signal interrupts to toggle at a given frequency (in hertz) and duration (in milliseconds).
	void tone(unsigned long frequency, unsigned long duration = 0);

	//Stop tone.
	void noTone();

	//Change an to a desired output pin. Stops the tone.
	void setOutputPin(byte output_pin);

	//Return true if instance was constructed successfully
	inline bool IsValid()
	{
		return is_valid;
	}
private:
	bool SetTimerPin(byte timer_pin);
	void HandleInterrupts();

	//Functor wrapper for the interrupt callback function of the servo controller
	class ToneInterrupt : public TimerInterruptCallback
	{
	public:
		//Create new callback with a pointer to the controller
		ToneInterrupt(TimerTone* controller)
		{
			controller_ = controller;
		}
		//When this interrupt is executed, handle the interrupts for this servo
		virtual void execute()
		{
			controller_->HandleInterrupts();
		}
	private:
		//Which servo controller this interrupt should act on
		TimerTone* controller_;
	};
	ToneInterrupt* interrupt_callback; //Pointer to interrupt_callback attached to this instance of TimerServo

	bool is_valid = false;
	long ocr;
	volatile long toggle_count;
	byte output_pin;
	byte timer_pin;
	volatile uint8_t *pin_port;
	volatile uint8_t pin_mask;
	volatile uint8_t *OCRnX;
	volatile uint8_t *TCNTn;
};

#endif