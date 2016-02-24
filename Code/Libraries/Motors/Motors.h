#ifndef Motors_H
#define Motors_H

#include "Arduino.h"
#include "Sensors.h"
#include "MotorDriver.h"
#include "Directions.h"
#include "Servo.h"
#include "math.h"

//Values used to configure the motors.
struct MotorConfig
{
	byte turn_deadzone; //How lenient we want our rotations to be
	byte drive_power; //power to the drivetrain

	byte victim_servo_pin;

	byte victim_servo_closed_angle;
	byte victim_servo_open_angle;

	unsigned long servo_close_time;
	unsigned long servo_open_time;

	float GYRODOMETRY_THRESHOLD;

	unsigned long PID_sample_time;
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
	bool pid_running = false;

	/**
	 * Functions
	 */
	//Constructor
	Motors(MotorConfig motor_config, Gyro* gyro, MotorDriver* motor_driver);

	//Destructor
	~Motors();

	//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
	bool Turn90(Direction dir);

	//Sets the constants for the PID controller as well as the desired sample time.
	void StartPID(float set_point, float input, bool reverse, bool inverse, float kp, float ki, float kd, unsigned long sample_time);

	//Sets the constants for the PID controller.
	void StartPID(float set_point, float input, bool reverse, bool inverse, float kp, float ki, float kd);

	//Signal that the PID is to be reset on the next run
	void StopPID();

	//Brakes the motors.
	void StopMotors();
	
	/**
	* Uses PID control to go forward. Given a current value (Gyro reading, for example), the function tries
	* to keep the robot aligned with the desired value passed into the function.
	* NOTE: THIS SHOULD ONLY CALLED BY INTERRUPT ROUTINE, NOT BY OTHER THINGS
	*/
	void GoUsingPIDControl();

	//Uses gyro and pid controlled motors to follow a heading.
	bool FollowHeading(float heading_deg, unsigned long desired_time_micros = 0UL, float desired_distance_mm = 0.0, bool reverse = false);

	//Uses encoders and PID control to go straight
	bool GoStraight(unsigned long desired_time_micros = 0UL, float desired_distance_mm = 0.0, bool reverse = false);

	//Combines the gyro and the encoders (gyrodometry) to get the degrees of the robot.
	float GetDegrees();

	//Get X position of robot in mm, based on encoders and the gyro.
	float GetX();

	//Get Y position of robot in mm, based on encoders and the gyro.
	float GetY();

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
	float gyrodometry_angle_ = 0.0;
	float X_pos = 0.0;
	float Y_pos = 0.0;
	float GYRODOMETRY_THRESHOLD;

	float set_point = 0.0;
	float input = 0.0;
	bool reverse = 0.0;
	bool inverse = 0.0;
	float kp = 0.0;
	float ki = 0.0;
	float kd = 0.0;
	unsigned long PID_sample_time_;
	short PID_out_max;
	short PID_out_min;
	volatile float previous_input_ = 0.0;
	volatile float integral_ = 0.0;

	unsigned long timer_ = 0UL;

	//config variables
	byte turn_deadzone_; //How lenient we want our rotations to be
	byte drive_power_; //power to the drivetrain

	byte victim_servo_closed_angle_;
	byte victim_servo_open_angle_;

	unsigned long servo_close_time_;
	unsigned long servo_open_time_;

	Servo victim_servo_;

	/**
	 * Functions
	 */

	/**
	 * Give both wheels power to rotate on a dime in a given direction.
	 * 0 <= power <= 255
	 */
	void TurnStationary(byte power, Direction dir);

	void UpdateGyrodometry();

	//Resets the saved values for the PID controller of the motors.
	//Takes a float input so that when it calculates the derivative there is no output spike.
	void ResetPID(float input);
};

#endif