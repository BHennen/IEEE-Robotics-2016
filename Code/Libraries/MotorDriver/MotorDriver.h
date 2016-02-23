#ifndef MotorDriver_h
#define MotorDriver_h

#include <Arduino.h>
#include <math.h>
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

	float LEFT_MMS_PER_TICK;
	float RIGHT_MMS_PER_TICK;
	float WHEELBASE;
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

	void UpdateOdometry();

	/*
	 * TODO: digitalRead is 29-50X slower than direct read of the port, but is more flexible.
	 * How to do direct read:
	 *
	 * bitRead(PIND, 0); //Reads pin 21
	 * bitRead(PIND, 1); //Reads pin 20
	 * bitRead(PIND, 2); //Reads pin 19
	 * bitRead(PIND, 3); //Reads pin 18
	 * bitRead(PINE, 4); //Reads pin 2
	 * bitRead(PINE, 5); //Reads pin 3
	 */

	/* On pinchange(A), if most recent value of pinA and previous value of pinB are both high or both low, it is spinning
	* clockwise. If they're different, it's going counterclockwise.
	* For left motor, CCW = forward
	*/
	inline void UpdateLeftEncoderA()
	{
		//Update value of A encoder before checking direction
		left_A_new = bitRead(*left_int_port_A, left_int_bit_A);
		//Compare A and B and update accordingly
		(left_A_new ^ left_B_old) ? left_encoder_ticks_++ : left_encoder_ticks_--;
	};

	/* On pinchange(B), if most recent value of pinA and previous value of pinB are both high or both low, it is spinning
	* clockwise. If they're different, it's going counterclockwise.
	* For left motor, CCW = forward
	*/
	inline void UpdateLeftEncoderB()
	{
		//Compare A and B and update accordingly
		(left_A_new ^ left_B_old) ? left_encoder_ticks_++ : left_encoder_ticks_--;
		//Update value of B encoder after checking direction
		left_B_old = bitRead(*left_int_port_B, left_int_bit_B);
	};

	/* On pinchange(A), if most recent value of pinA and previous value of pinB are both high or both low, it is spinning
	* clockwise. If they're different, it's going counterclockwise.
	* For right motor, CW = forward
	*/
	inline void UpdateRightEncoderA()
	{
		//Update value of A encoder before checking direction
		right_A_new = bitRead(*right_int_port_A, right_int_bit_A);
		//Compare A and B and update accordingly
		(right_A_new ^ right_B_old) ? right_encoder_ticks_-- : right_encoder_ticks_++;
	};

	/* On pinchange(B), if most recent value of pinA and previous value of pinB are both high or both low, it is spinning
	* clockwise. If they're different, it's going counterclockwise.
	* For right motor, CW = forward
	*/
	inline void UpdateRightEncoderB()
	{
		//Compare A and B and update accordingly
		(right_A_new ^ right_B_old) ? right_encoder_ticks_-- : right_encoder_ticks_++;
		//Update value of B encoder after checking direction
		right_B_old = bitRead(*right_int_port_B, right_int_bit_B);
	};

	long prev_left_ticks_ = 0;
	long prev_right_ticks_ = 0;

	float LEFT_MMS_PER_TICK;
	float RIGHT_MMS_PER_TICK;
	float WHEELBASE;

	volatile long left_encoder_ticks_ = 0;
	volatile long right_encoder_ticks_ = 0;

private:
	byte enable_pin_;
	byte fault_pin_;
	byte left_motor_pin_fwd_;
	byte left_motor_pin_bwd_;
	byte left_motor_current_pin_;
	byte right_motor_pin_fwd_;
	byte right_motor_pin_bwd_;
	byte right_motor_current_pin_;

	volatile byte* left_int_port_A;
	byte left_int_bit_A;
	volatile byte* left_int_port_B;
	byte left_int_bit_B;
	volatile byte* right_int_port_A;
	byte right_int_bit_A;
	volatile byte* right_int_port_B;
	byte right_int_bit_B;

	volatile byte left_A_new = 0;
	volatile byte left_B_old = 0;
	volatile byte right_A_new = 0;
	volatile byte right_B_old = 0;

	float theta = 0.0; /* bot heading */
	float X_pos = 0.0; /* bot X position in mms */
	float Y_pos = 0.0; /* bot Y position in mms */
};

#endif