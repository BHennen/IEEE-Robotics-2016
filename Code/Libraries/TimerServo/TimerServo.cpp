#include <TimerServo.h>

#define usToOCR(_us)    roundDivide((clockCyclesPerMicrosecond() * (_us)), 8)     // converts microseconds to ocr (assumes prescale of 8)

//Create a controller for a group of 8 servos on a pin. Assumes a timer has already been set with Timer::SetMode(...).
//Also assumes we're using a 16 bit timer with a prescale of 8.
TimerServo::TimerServo(byte timer_pin)
{
	this->timer_pin = timer_pin;
	uint8_t timer = digitalPinToTimer(timer_pin);
	if(testTimer8bit(timer))
	{
		//Do not use 8 bit timers for servo (for now)
		return;
	}

	//Set the register for the output compare value
	this->OCRnX = timerToOCRnX(timer);
	if(OCRnX == NOT_ON_TIMER)
	{
		return;
	}

	//Set the register for the timer count value
	this->TCNTn = timerToTCNTn(timer);
	if(TCNTn == NOT_ON_TIMER)
	{
		return;
	}

	is_valid = true;
}

// attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
uint8_t TimerServo::attach(byte pin, uint16_t min, uint16_t max, byte channel /*= INVALID_SERVO*/)
{
	//Find channel automatically
	if(channel == INVALID_SERVO)
	{
		channel = 0;
		//Loop through all servos. Stop if we find an inactive one or we've gone past the channel limits
		while(channel < SERVOS_PER_TIMER && servos[channel].Pin.isActive)
		{
			channel++;
		}
	}
	//Attach this pin to a channel
	if(channel < SERVOS_PER_TIMER)
	{
		servo_t& servo = servos[channel];
		servo.Pin.nbr = pin; //set output pin number
		pinMode(pin, OUTPUT); // set servo pin to output
		servo.ocr = usToOCR(DEFAULT_PULSE_WIDTH); // set default ocr
		
		//Set min/max
		int8_t min_res = (MIN_PULSE_WIDTH - min) / 4; //resolution of min/max is 4 uS
		servo.min = MIN_PULSE_WIDTH - min_res * 4;
		int8_t max_res = (MAX_PULSE_WIDTH - max) / 4;
		servo.max = MAX_PULSE_WIDTH - max_res * 4;

		//Enable interrupt for this timer if this is the first active channel, and set this channel to be active.
		if(!isActive())
		{
			Timer::EnableInterrupt(timer_pin);
		}
		//Make sure the addition of a new channel is atomic; all necessary values added at once
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			totalOCR += servo.ocr; //keep track of total ocr
			servo.Pin.isActive = true;
		}
	}
	else //Requested channel is more than maximum allowed
	{
		channel = INVALID_SERVO;
	}
	return channel;
}

// as above but sets min and max values to default. 
uint8_t TimerServo::attach(byte pin, byte channel /*= INVALID_SERVO*/)
{
	return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, channel);
}

void TimerServo::detach(byte channel)
{
	if(channel >= 0 && channel < SERVOS_PER_TIMER)
	{
		//Disable servo (and interrupt, if all servos on this timer are disabled).
		servo_t& servo = servos[channel];
		//Make sure the removal of a channel is atomic; all necessary values removed at once
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			totalOCR -= servo.ocr; //keep track of total ocr
			servo.Pin.isActive = false;
		}
		if(isActive() == false)
			Timer::DisableInterrupt(timer_pin);
	}
}

void TimerServo::write(byte value, byte channel)
{
	if(channel >= 0 && channel < SERVOS_PER_TIMER)
	{
		servo_t& servo = servos[channel];
		if(value < 0) value = 0;
		if(value > 180) value = 180;
		value = map(value, 0, 180, servo.min, servo.max);
		this->writeMicroseconds(value, channel);
	}
}

// Write pulse width in microseconds	
void TimerServo::writeMicroseconds(uint16_t value, byte channel)
{
	// calculate and store the values for this servo
	// ensure channel is valid
	if(channel >= 0 && channel < SERVOS_PER_TIMER)
	{
		servo_t& servo = servos[channel];
		if(value < servo.min)          // ensure pulse width is valid
			value = servo.min;
		else if(value > servo.max)
			value = servo.max;

		value = value - TRIM_DURATION;
		value = usToOCR(value);  // convert to ticks after compensating for interrupt overhead

		//Make sure the write is an atomic operation.
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			totalOCR += value; //add new ocr
			totalOCR -= servo.ocr; //subtract old ocr
			servo.ocr = value;
		}
	}
}

bool TimerServo::isActive()
{
	// returns true if any servo is active on this timer
	for(uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++)
	{
		if(servos[channel].Pin.isActive)
			return true;
	}
	return false;
}

void TimerServo::HandleInterrupts()
{	
	if(current_channel == -1) //current_channel set to -1 to signal end of refresh interval; set nothing low
	{		
	}
	else //Set previously high channel low
	{
		digitalWrite(servos[current_channel].Pin.nbr, LOW);
	}

	//Loop through all channels. Stop if we find an active one or we've gone past the channel limits
	do
	{
		current_channel++; //move to next channel.
	}
	while(current_channel < SERVOS_PER_TIMER && !servos[current_channel].Pin.isActive);

	//We found an active pin
	if(current_channel < SERVOS_PER_TIMER)
	{
		//Set this pin high
		digitalWrite(servos[current_channel].Pin.nbr, HIGH);
		//Call the interrupt again in OCR ticks to set the pin low.
		*OCRnX = *TCNTn + servos[current_channel].ocr;
	}
	else //No more active pins were found; wait for the refresh time & set current_channel to -1
	{
		//Remaining time until refresh == RefreshInterval - totalOCR
		*OCRnX = *TCNTn + (REFRESH_INTERVAL - totalOCR);
		current_channel = -1;
	}
}