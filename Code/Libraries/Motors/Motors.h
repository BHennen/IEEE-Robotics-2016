#ifndef Motors_H
#define Motors_H

#include "Arduino.h"
#include "Sensors.h"
#include "MotorDriver.h"

//Values used to configure the motors.
struct MotorConfig
{
	byte left_motor_pin_fwd;
	byte left_motor_pin_bwd;
	byte left_motor_current_pin;
	byte right_motor_pin_fwd;
	byte right_motor_pin_bwd;
	byte right_motor_current_pin;
	byte enable_pin;
	byte fault_pin;

	byte turn_deadzone; //How lenient we want our rotations to be
	byte drive_power; //power to the drivetrain
};

//Directions to turn, wall follow, etc
enum Direction
{
	LEFT,
	RIGHT
};

/**
 * Class that contains Motors to act on the world.
 */
class Motors
{
public:
	
	/**
	 * Variables
	 */

	MotorConfig config;

	/**
	 * Functions
	 */

	//Constructor
	Motors(MotorConfig motor_config, Gyro* gyro);

	//Destructor
	~Motors();

	//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
	bool Turn90(Direction dir);

	//Resets the saved values for the PID controller of the motors
	void ResetPID();

	/**
	* Uses PID control to go forward. Given a current value (Gyro reading, for example), the function tries
	* to keep the robot aligned with the desired value passed into the function.
	* *** CRITICAL: Before using function ResetPID() must be ***
	* *** called (only once) to clear saved variable values. ***
	*/
	void GoUsingPIDControl(float desired_value, float current_value, float kp, float ki, float kd);

	//Brakes the motors.
	void StopMotors();

	/**
	* Uses gyro and pid controlled motors to follow a heading.
	* *** CRITICAL: Before using function ResetPID() must be ***
	* *** called(only once) to clear saved variable values.  ***
	*/
	bool FollowHeading(float heading_deg, unsigned long desired_time_micros = 0UL);

private:
	/**
	 * Variables
	 */
	Gyro *gyro_;
	MotorDriver *drivetrain_;

	bool rotating_ = false;
	float desired_degrees_ = 0.0;

	unsigned long previous_time_ = 0UL;
	float previous_error_ = 0.0;
	float integral_ = 0.0;
	/**
	 * Functions
	 */

	/**
	 * Give both wheels power to rotate on a dime in a given direction.
	 * 0 <= power <= 255
	 */
	void TurnStationary(byte power, Direction dir);

};

#endif