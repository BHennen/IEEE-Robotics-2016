#ifndef Motors_H
#define Motors_H

#include <Arduino.h>

//Values used to configure the motors.
struct MotorConfig
{
	byte left_motor_pin_fwd;
	byte left_motor_pin_bwd;
	byte right_motor_pin_fwd;
	byte right_motor_pin_bwd;

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
	Motors(MotorConfig config, Gyro* gyro);
		
	//Destructor
	~Motors();

	//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
	bool Turn90(Direction d, unsigned long current_time);

	//Resets the saved values for the PID controller of the motors
	void ResetPID();

	/**
	* Uses PID control to go forward. Given a current value (Gyro reading, for example), the function tries
	* to keep the robot aligned with the desired value passed into the function.
	* *** CRITICAL: Before using function ResetPID() must be ***
	* *** called (only once) to clear saved variable values. ***
	*/
	void GoUsingPIDControl(int desired_value, int current_value, int* pid_consts);

	//Brakes the motors.
	void StopMotors();

private:
	/**
	 * Variables
	 */
	Gyro *gyro_;

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