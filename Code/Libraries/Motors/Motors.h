#ifndef Motors_H
#define Motors_H

#include "Arduino.h"
#include "Sensors.h"
#include "MotorDriver.h"
#include "Directions.h"
#include "Servo.h"

//Values used to configure the motors.
struct MotorConfig
{
	byte turn_deadzone; //How lenient we want our rotations to be
	byte drive_power; //power to the drivetrain

	byte left_servo_pin;
	byte right_servo_pin;

	byte left_servo_closed_angle;
	byte right_servo_closed_angle;
	byte left_servo_open_angle;
	byte right_servo_open_angle;

	byte servo_close_time;
	byte servo_open_time;
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
	MotorDriver *drivetrain;

	/**
	 * Functions
	 */

	//Constructor
	Motors(MotorConfig motor_config, Gyro* gyro, MotorDriver* motor_driver);

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

	//Close servos to grab the victim
	bool BiteVictim();

	//Open servos to release the victim
	bool ReleaseVictim();


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
	unsigned long timer_ = 0UL;

	//config variables
	byte turn_deadzone_; //How lenient we want our rotations to be
	byte drive_power_; //power to the drivetrain

	byte left_servo_closed_angle_;
	byte right_servo_closed_angle_;
	byte left_servo_open_angle_;
	byte right_servo_open_angle_;

	unsigned long servo_close_time_;
	unsigned long servo_open_time_;

	Servo left_servo_;
	Servo right_servo_;

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