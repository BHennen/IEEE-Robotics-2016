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

	/**
	 * Functions
	 */

	//Constructor
	Motors(MotorConfig config, Gyro* gyro);
		
	//Destructor
	~Motors();

	//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
	bool Turn90(Direction d, unsigned long current_time);

	//Uses PID control to go forward, trying to keep the robot aligned with the desired value passed into the function.
	void GoUsingPIDControl(int desired_value, int current_value, int* pid_consts, unsigned long current_time);
	
	//Goes straight using the gyro or encoders (or both).
	void GoStraight();

	//Brakes the motors.
	void StopMotors();

private:
	/**
	 * Variables
	 */

	MotorConfig motor_config_;
	Gyro *gyro_;

	bool rotating_ = false;
	float desired_degrees_ = 0.0;

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