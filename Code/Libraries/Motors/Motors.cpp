#include "Motors.h"
#include "Sensors.h"


/**
* Constructor.
*/
Motors::Motors(MotorConfig config, Gyro* gyro)
{
	motor_config_ = config;
	pinMode(motor_config_.left_motor_pin_fwd, OUTPUT);
	pinMode(motor_config_.left_motor_pin_bwd, OUTPUT);
	pinMode(motor_config_.right_motor_pin_fwd, OUTPUT);
	pinMode(motor_config_.right_motor_pin_bwd, OUTPUT);

	gyro_ = gyro;
}

//Destructor
Motors::~Motors()
{

}

//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
bool Motors::Turn90(Direction dir)
{
	//Code that uses only the gyro
	float current_degrees = gyro_->getDegrees();
	//If the robot is not currently rotating and this method is called
	//determine the values needed for the upcoming rotation
	if(!rotating_)
	{
		//Set the robots desired degrees based on current degrees
		float required_angle = (dir == RIGHT) ? 90 : -90;
		desired_degrees_ = current_degrees + required_angle;
		
		//Set desiredDegrees so that it is <180 and >=0
		if(desired_degrees_ >= 360)
		{
			desired_degrees_ -= 360;
		}
		else if(desired_degrees_ < 0)
		{
			desired_degrees_ += 360;
		}
	}
	else
	{
		//Robot is currently rotating, values not needed to be computed
	}
	
	float diff = desired_degrees_ - current_degrees;
	if(diff > 180.0f) diff -= 360;
	if(diff < -180.0f) diff += 360;
	if(abs(diff) < motor_config_.turn_deadzone)
	{
		//Robot has rotated the correct amount
		StopMotors(); //brake motors
		rotating_ = false;
		return true;
	}
	else //Robot has not rotated the correct amount, continue rotating
	{
		//turn based on the difference (so if we overshoot it will turn correct way)
		Direction turn_direction = (diff > 0) ? RIGHT : LEFT;
		TurnStationary(motor_config_.drive_power, turn_direction);

		rotating_ = true;
		return false;
	}
}

//Uses PID control to go forward, trying to keep the robot aligned with the desired value passed into the function.
void Motors::GoUsingPIDControl(int desired_value, int current_value, int* pid_consts, unsigned long current_time)
{

}

//Goes straight using the gyro or encoders (or both).
void Motors::GoStraight()
{

}

/**
* Give both wheels power to rotate on a dime.
* 0 <= power <= 255
*/
void Motors::TurnStationary(byte power, Direction dir)
{
	if(dir == RIGHT) //rotate right
	{
		analogWrite(motor_config_.left_motor_pin_fwd, power);
		analogWrite(motor_config_.left_motor_pin_bwd, 0);
		analogWrite(motor_config_.right_motor_pin_fwd, 0);
		analogWrite(motor_config_.right_motor_pin_bwd, power);
	}
	else //rotate left
	{
		analogWrite(motor_config_.left_motor_pin_fwd, 0);
		analogWrite(motor_config_.left_motor_pin_bwd, power);
		analogWrite(motor_config_.right_motor_pin_fwd, power);
		analogWrite(motor_config_.right_motor_pin_bwd, 0);
	}
}

//Brakes the motors.
void Motors::StopMotors()
{
	analogWrite(motor_config_.left_motor_pin_fwd, 100);
	analogWrite(motor_config_.left_motor_pin_bwd, 100);
	analogWrite(motor_config_.right_motor_pin_fwd, 100);
	analogWrite(motor_config_.right_motor_pin_bwd, 100);
}