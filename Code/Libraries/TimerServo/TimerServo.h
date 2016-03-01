/*
  Influenced by Arduino Servo library.

  A servo is activated by creating an instance of the Servo class, passing
  the desired timer_pin to the constructor. This library is dependent on the 
  TimerHelper library. It is assumed a timer was previously enabled using the
  TimerHelper library function Timer::SetMode(...). You then need to attach the 
  timer interrupt to the same timer pin and the instance of your Timer Servo. An 
  example is as follows:


  ...
  TimerServo* servo_controller;
  void ServoControllerISR() { servo_controller->HandleInterrupts(); }
  ...
  Timer::SetMode(PID_timer_pin,							//Same pin group as the TimerServo's pin
				 Timer::MODE::NORMAL,					//normal mode required
				 Timer::PRESCALE_8,						//Prescale by 8 required
				 Timer::PORT_OUTPUT_MODE::NO_OUTPUT		//No output required
  ...
  servo_controller = new TimerServo(39); //Creates a timer to control up to 8 servos for a given timer pin
  if(!servo_controller->IsValid())
  {
	//Make sure to handle the case where the pin number is invalid
  }
  //Attach timer interrupt to the controller only if initialized correctly, but don't enable interrupts
  Timer::AttachInterrupt(39, ServoControllerISR, false);
  ...


  The servos are then pulsed in the background using the value most
  recently written using the write() method.

  Note that analogWrite of PWM on pins associated with the timer in
  SetMode(...) are disabled.

  The methods are:

	TimerServo - Class for manipulating servo motors connected to a specific Arduino timer pin.

	attach(pin, channel = AUTO )  - Attaches a servo motor to an i/o pin. Optionally specify a desired channel. Return channel it was assigned.
	attach(pin, min, max, channel = AUTO ) - Same as above, but also setting min and max values in microseconds
	default min is 544, max is 2400

	write(channel )     - Sets the servo angle in degrees.
	writeMicroseconds(channel ) - Sets the servo pulse width in microseconds.
	detach(channel)    - Stops an attached servo from pulsing its i/o pin.

	isActive()		- Returns true if there is at least one active channel.
	IsValid()		- Returns true if constructor completed successfully.

	HandleInterrupts()	- Interrupt routine that should be called in a Timer::AttachInterrupt callback.
 */

#ifndef TimerServo_h
#define TimerServo_h

#include <TimerHelper.h>

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds 

#define SERVOS_PER_TIMER       8     // the maximum number of servos controlled by one timer 

constexpr uint8_t INVALID_SERVO = 255;     // flag indicating an invalid servo index

#define TRIM_DURATION       2         // compensation ticks to trim adjust for digitalWrite delays

typedef struct
{
	uint8_t nbr :6;      // a pin number from 0 to 63
	uint8_t isActive :1; // true if active, false if not active
} ServoPin_t;

typedef struct
{
	int8_t min;
	int8_t max;
	volatile unsigned int ocr;
	ServoPin_t Pin;
} servo_t;

//class ServoInterrupt; //forward definition

class TimerServo
{
public:
	//Create a controller for a group of 8 servos on a pin
	TimerServo(byte timer_pin);
	~TimerServo();
	//Uses this timer to control the given pin. Optionally lets user pick which channel the servo is on.
	//Returns the channel the pin is attached to, or INVALID_SERVO if error
	uint8_t attach(byte pin, byte channel = INVALID_SERVO);
	// as above but also sets min and max values for writes. 
	uint8_t attach(byte pin, uint16_t min, uint16_t max, byte channel = INVALID_SERVO);
	//Detach a channel from the servo
	void detach(byte channel);
	//Write angle of servo in degrees to a channel
	void write(byte value, byte channel); 
	// Write pulse width in microseconds  to a channel
	void writeMicroseconds(uint16_t value, byte channel);
	//Check if at least one channel is active on this TimerServo
	bool isActive();

	//Return true if instance was constructed successfully
	inline bool IsValid()
	{
		return is_valid;
	}

private:
	//Interrupt routine to run this TimerServo's array of servos.
	void HandleInterrupts();

	//Functor wrapper for the interrupt callback function of the servo controller
	class ServoInterrupt : public TimerInterruptCallback
	{
	public:
		//Create new callback with a pointer to the controller
		ServoInterrupt(TimerServo* controller)
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
		TimerServo* controller_;
	};
	ServoInterrupt* interrupt_callback; //Pointer to interrupt_callback attached to this instance of TimerServo

	//Which channel to execute next in the ISR
	volatile int8_t current_channel = -1;

	//timer count and compare count
	volatile uint8_t* OCRnX;
	volatile uint8_t* TCNTn;

	bool is_valid = false;//True if constructor completed successfully

	uint16_t totalOCR; //How many total ticks are required by all channels

	uint8_t timer_pin; //Which timer pin the TimerServo is tied to.

	//Array of servos and their configurations
	servo_t servos[SERVOS_PER_TIMER];
};

#endif
