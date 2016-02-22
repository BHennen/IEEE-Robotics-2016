#ifndef MotorDriver_h
#define MotorDriver_h

#include <Arduino.h>

/**
 * Code heavily edited from Pololu: https://github.com/pololu/dual-mc33926-motor-shield
 * BUT we are using the Dual MC33926 Motor Driver Carrier, not the shield, so it requires some modifications:
 *
 * Mappings from the motor driver to the arduino to the code are as follows:
 * M1IN1 ---> Arduino PWM Capable pin ---------------> left_motor_pin_fwd
 * M1IN2 ---> Arduino PWM Capable pin ---------------> left_motor_pin_bwd
 * M1FB ----> Arduino Analog In pin -----------------> left_motor_current_pin
 * M1SF ----> Tied with M2SF; Arduino Digital pin ---> fault_pin
 * M2IN1 ---> Arduino PWM Capable pin ---------------> right_motor_pin_fwd
 * M2IN2 ---> Arduino PWM Capable pin ---------------> right_motor_pin_bwd
 * M2FB ----> Arduino Analog In pin -----------------> right_motor_current_pin
 * M2SF ----> Tied with M1SF; Arduino Digital pin ---> fault_pin
 * EN ------> Arduino Digital pin -------------------> enable_pin
 *
 * VDD -----> Arduino 5 V
 * M1D1 ----> GND jumper
 * M1D2 ----> VDD jumper
 * M2D1 ----> GND jumper
 * M2D2 ----> VDD jumper
 * VIN -----> Power Source (+)
 * GND -----> Power Source (-)
 * M1OUT1 and M1OUT2 to Motor 1
 * M2OUT1 and M2OUT2 to Motor 2
 *
 * Others can be ignored.
 */

//Values used to configure motor driver.
struct MotorDriverConfig
{
	byte left_motor_pin_fwd;
	byte left_motor_pin_bwd;
	byte left_motor_current_pin;
	byte right_motor_pin_fwd;
	byte right_motor_pin_bwd;
	byte right_motor_current_pin;
	byte enable_pin;
	byte fault_pin;

	byte left_motor_encoder_A;
	byte left_motor_encoder_B;
	byte right_motor_encoder_A;
	byte right_motor_encoder_B;
};

class MotorDriver
{
public:
	// CONSTRUCTOR
	MotorDriver(MotorDriverConfig motor_driver_config);

	// PUBLIC METHODS
	void SetLeftSpeed(int speed); // Set speed for Left Motor.
	void SetRightSpeed(int speed); // Set speed for Right Motor.
	void SetSpeeds(int left_speed, int right_speed); // Set speed for both Left Motor and Right Motor.
	unsigned int GetLeftCurrentMilliamps(); // Get current reading for Left Motor. 
	unsigned int GetRightCurrentMilliamps(); // Get current reading for Right Motor.
	bool isFault(); // Get fault reading.
	
	/* On pinchange(A), if pinA and pinB are both high or both low, it is spinning
	* clockwise. If they're different, it's going counterclockwise.
	* For left motor, CCW = forward
	*/
	inline void UpdateLeftEncoderA()
	{
		if(digitalRead(left_motor_encoder_A_) == digitalRead(left_motor_encoder_B_))
		{
			left_encoder_ticks_--;
		}
		else
		{
			left_encoder_ticks_++;
		}
	};

	/* On pinchange(B), if pinA and pinB are both high or both low, it is spinning
	* counterclockwise. If they're different, it's going clockwise.
	* For left motor, CCW = forward
	*/
	inline void UpdateLeftEncoderB()
	{
		if(digitalRead(left_motor_encoder_A_) == digitalRead(left_motor_encoder_B_))
		{
			left_encoder_ticks_++;
		}
		else
		{
			left_encoder_ticks_--;
		}
	};

	/* On pinchange(A), if pinA and pinB are both high or both low, it is spinning
	* clockwise. If they're different, it's going counterclockwise.
	* For right motor, CW = forward
	*/
	inline void UpdateRightEncoderA()
	{
		if(digitalRead(right_motor_encoder_A_) == digitalRead(right_motor_encoder_B_))
		{
			right_encoder_ticks_++;
		}
		else
		{
			right_encoder_ticks_--;
		}
	};

	/* On pinchange(B), if pinA and pinB are both high or both low, it is spinning
	* counterclockwise. If they're different, it's going clockwise.
	* For right motor, CW = forward
	*/
	inline void UpdateRightEncoderB()
	{
		if(digitalRead(right_motor_encoder_A_) == digitalRead(right_motor_encoder_B_))
		{
			right_encoder_ticks_--;
		}
		else
		{
			right_encoder_ticks_++;
		}
	};

private:
	byte enable_pin_;
	byte fault_pin_;
	byte left_motor_pin_fwd_;
	byte left_motor_pin_bwd_;
	byte left_motor_current_pin_;
	byte right_motor_pin_fwd_;
	byte right_motor_pin_bwd_;
	byte right_motor_current_pin_;

	byte left_motor_encoder_A_;
	byte left_motor_encoder_B_;
	byte right_motor_encoder_A_;
	byte right_motor_encoder_B_;

	volatile long left_encoder_ticks_;
	volatile long right_encoder_ticks_;
};

#endif