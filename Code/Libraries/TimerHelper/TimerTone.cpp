#include "TimerTone.h"

//Create a new TimerTone instance which can generate
//a single tone at a time using the specified timer and
//output pins.
TimerTone::TimerTone(byte timer_pin, byte output_pin)
{
	if(!SetTimerPin(timer_pin))
	{
		return;
	}

	this->output_pin = output_pin;
	pin_port = portOutputRegister(digitalPinToPort(output_pin));
	pin_mask = digitalPinToBitMask(output_pin);

	is_valid = true;
}

// frequency (in hertz) and duration (in milliseconds).
void TimerTone::tone(unsigned long frequency, unsigned long duration /*= 0*/)
{
	//Get correct OCR. 2*frequency because we need 50% duty cycle
	long tmp = Timer::GetOCR(2*frequency, timer_pin, Timer::CLOCK::PRESCALE_8); //FIXME: Assume prescale of 8 for now
	if(tmp == -1) return; //invalid ocr
	ocr = tmp;

	// Calculate the toggle count
	if(duration > 0)
	{
		toggle_count = 2 * frequency * duration / 1000;
	}
	else
	{
		toggle_count = -1;
	}

	// Set the pinMode as OUTPUT
	pinMode(output_pin, OUTPUT);

	// Enable interrupts
	Timer::EnableInterrupt(timer_pin);
}

//Stop the tone from playing.
void TimerTone::noTone()
{
	//Stop interrupts and set pin low
	Timer::DisableInterrupt(timer_pin);
	digitalWrite(output_pin, LOW);
}

//Stops the current tone and changes to the new output pin.
void TimerTone::setOutputPin(byte output_pin)
{
	noTone(); //stop previous tone
	//Update new pin info
	this->output_pin = output_pin;
	pin_port = portOutputRegister(digitalPinToPort(output_pin));
	pin_mask = digitalPinToBitMask(output_pin);
}

//Private function to initialize the timer & interrupt
bool TimerTone::SetTimerPin(byte timer_pin)
{	
	uint8_t timer = digitalPinToTimer(timer_pin);
	if(testTimer8bit(timer))
	{
		//Do not use 8 bit timers for servo (for now)
		return false;
	}

	//Set the register for the output compare value
	volatile uint8_t *OCRnX = timerToOCRnX(timer);
	if(OCRnX == NOT_ON_TIMER)
	{
		return false;
	}

	//Set the register for the timer count value
	volatile uint8_t *TCNTn = timerToTCNTn(timer);
	if(TCNTn == NOT_ON_TIMER)
	{
		return false;
	}

	ToneInterrupt* interrupt_cb = new ToneInterrupt(this); //Create interrupt callback
	if(!Timer::AttachInterrupt(timer_pin, interrupt_cb, false)) //Attach interrupt to timer, but do not enable yet
	{
		//delete callback and return if interrupt not attached successfully.
		delete interrupt_cb;
		Serial.println(F("Error attaching callback - TimerServo"));
		return false;
	}

	//All pin checks completed successfully; assign new stuff
	this->timer_pin = timer_pin;
	this->OCRnX = OCRnX;
	this->TCNTn = TCNTn;
	this->interrupt_callback = interrupt_cb;
	return true;
}

//Toggles the pin every 1/2 frequency
void TimerTone::HandleInterrupts()
{
	//If there is duration remaining
	if(toggle_count != 0)
	{
		// toggle the pin
		*pin_port ^= pin_mask;

		//Decrement duration
		if(toggle_count > 0)
			toggle_count--;

		//Set ocr so that the interrupt is called again
		*OCRnX = *TCNTn + ocr;
	}
	else //No duration left; stop tone
	{
		noTone();
	}
}