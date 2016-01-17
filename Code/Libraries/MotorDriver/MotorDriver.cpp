#include "MotorDriver.h"

/*
* Constructor. Mappings from the motor driver to the arduino to the code are as follows:
* M1IN1 ---> Arduino PWM Capable pin ---------------> left_motor_pin_fwd
* M1IN2 ---> Arduino PWM Capable pin ---------------> left_motor_pin_bwd
* M1FB ----> Arduino Analog In pin -----------------> left_motor_current_pin
* M1SF ----> Tied with M2SF; Arduino Digital pin ---> fault_pin
* M2IN1 ---> Arduino PWM Capable pin ---------------> right_motor_pin_fwd
* M2IN2 ---> Arduino PWM Capable pin ---------------> right_motor_pin_bwd
* M2FB ----> Arduino Analog In pin -----------------> right_motor_current_pin
* M2SF ----> Tied with M1SF; Arduino Digital pin ---> fault_pin
* EN ------> Arduino Digital pin -------------------> enable_pin
*/
MotorDriver::MotorDriver(MotorDriverConfig motor_driver_config)
{
	//Pin map
	enable_pin_ = motor_driver_config.enable_pin;
	fault_pin_ = motor_driver_config.fault_pin;
	left_motor_pin_fwd_ = motor_driver_config.left_motor_pin_fwd;
	left_motor_pin_bwd_ = motor_driver_config.left_motor_pin_bwd;
	left_motor_current_pin_ = motor_driver_config.left_motor_current_pin;
	right_motor_pin_fwd_ = motor_driver_config.right_motor_pin_fwd;
	right_motor_pin_bwd_ = motor_driver_config.right_motor_pin_bwd;
	right_motor_current_pin_ = motor_driver_config.right_motor_current_pin;

	//Set pinModes
	pinMode(left_motor_pin_fwd_, INPUT);
	pinMode(left_motor_pin_bwd_, INPUT);
	pinMode(left_motor_current_pin_, INPUT);
	pinMode(right_motor_pin_fwd_, INPUT);
	pinMode(right_motor_pin_bwd_, INPUT);
	pinMode(right_motor_current_pin_, INPUT);
	pinMode(enable_pin_, OUTPUT);
	pinMode(fault_pin_, INPUT);
	digitalWrite(enable_pin_, HIGH); // default to on
}

// Public Methods //////////////////////////////////////////////////////////////

// Set speed for left motor, speed is a number betwenn -255 and 255
void MotorDriver::SetLeftSpeed(int speed)
{
	bool reverse = false;

	if(speed < 0)
	{
		speed = -speed;  // Make speed a positive quantity
		reverse = 1;  // Preserve the direction
	}
	if(speed > 255) // Max PWM dutycycle
		speed = 255;

	if(!reverse)
	{
		analogWrite(left_motor_pin_fwd_, speed);
		analogWrite(left_motor_pin_bwd_, 0);
	}
	else
	{
		analogWrite(left_motor_pin_fwd_, 0);
		analogWrite(left_motor_pin_bwd_, speed);
	}
}

// Set speed for right motor, speed is a number betwenn -255 and 255
void MotorDriver::SetRightSpeed(int speed)
{
	bool reverse = false;

	if(speed < 0)
	{
		speed = -speed;  // Make speed a positive quantity
		reverse = 1;  // Preserve the direction
	}
	if(speed > 255) // Max PWM dutycycle
		speed = 255;

	if(!reverse)
	{
		analogWrite(right_motor_pin_fwd_, speed);
		analogWrite(right_motor_pin_bwd_, 0);
	}
	else
	{
		analogWrite(right_motor_pin_fwd_, 0);
		analogWrite(right_motor_pin_bwd_, speed);
	}
}

// Set speed for left and right motors
void MotorDriver::SetSpeeds(int left_speed, int right_speed)
{
	SetLeftSpeed(left_speed);
	SetRightSpeed(right_speed);
}

// Return motor 1 current value in milliamps.
unsigned int MotorDriver::GetLeftCurrentMilliamps()
{
	// 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
	return analogRead(left_motor_current_pin_) * 9;
}

// Return motor 2 current value in milliamps.
unsigned int MotorDriver::GetRightCurrentMilliamps()
{
	// 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
	return analogRead(right_motor_current_pin_) * 9;
}

// Return error status
bool MotorDriver::isFault()
{
	return !digitalRead(fault_pin_);
}