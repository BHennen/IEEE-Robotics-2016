#include "Motors.h"
#include "Sensors.h"


/**
* Constructor.
*/
Motors::Motors(MotorConfig motor_config, Gyro* gyro, MotorDriver* motor_driver)
{
	drivetrain = motor_driver;

	gyro_ = gyro;

	turn_deadzone_ = motor_config.turn_deadzone;
	drive_power_ = motor_config.drive_power;

	victim_servo_.attach(motor_config.victim_servo_pin);

	victim_servo_closed_angle_ = motor_config.victim_servo_closed_angle;
	victim_servo_open_angle_ = motor_config.victim_servo_open_angle;

	servo_close_time_ = motor_config.servo_close_time;
	servo_open_time_ = motor_config.servo_open_time;

	GYRODOMETRY_THRESHOLD = motor_config.GYRODOMETRY_THRESHOLD;
}

//Destructor
Motors::~Motors()
{

}

//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
bool Motors::Turn90(Direction dir)
{
	//Code that uses the gyrodometery
	float current_degrees = GetDegrees();
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
	if(diff > 180.0f) 
		diff -= 360;
	else if(diff < -180.0f) 
		diff += 360;
	if(abs(diff) < turn_deadzone_)
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
		//Map the output power based on how far we are from the desired direction.
		//We rotate faster when further away
		byte power = map(abs(diff), 0, 180, drive_power_ / 4, drive_power_);
		TurnStationary(power, turn_direction);

		rotating_ = true;
		return false;
	}
}

//Resets the saved values for the PID controller of the motors
void Motors::ResetPID()
{
	integral_ = 0;
	previous_error_ = 0.0;
	previous_time_ = micros();
}

/**
 * Uses PID control to go forward. Given a current value (Gyro reading, for example), the function tries
 * to keep the robot aligned with the desired value passed into the function.
 * *** CRITICAL: Before using function ResetPID() must be ***
 * *** called (only once) to clear saved variable values. ***
 */
void Motors::GoUsingPIDControl(float desired_value, float current_value, float kp, float ki, float kd)
{
	//Determine PID output
	//Find how long has passed since the last adjustment.
	long dt = 0;
	unsigned long current_time = micros();
	dt = current_time - previous_time_;
	previous_time_ = current_time;
	if(dt == 0) return; //Only adjust if time has passed

	//Determine error; how far off the robot is from desired value
	float error = desired_value - current_value;

	//Determine integral; sum of all errors
	integral_ += error * (dt / 1000000.0f); //Divide by 1000000.0 because dt is microseconds, adjust for seconds

	//Determine derivative; rate of change of errors
	float derivative = (error - previous_error_) * (1000000.0f / dt); //Multiply by 1000000.0 because dt is microseconds, adjust for seconds

	//Determine output
	int output = (int)(kp * error + ki * integral_ + kd * derivative);

	//Save current error for next time
	previous_error_ = error;

	//Go with the adjusted power values.
	//Before adjustment for PWM limits
	int left_power = drive_power_ + output;
	int right_power = drive_power_ - output;

	//Go forward with new adjustments
	drivetrain->SetSpeeds(left_power, right_power);
}

/**
* Give both wheels power to rotate on a dime.
* 0 <= power <= 255
*/
void Motors::TurnStationary(byte power, Direction dir)
{
	if(dir == RIGHT) //rotate right
	{
		drivetrain->SetSpeeds(power, -power);
	}
	else //rotate left
	{
		drivetrain->SetSpeeds(-power, power);
	}
}

//Brakes the motors.
void Motors::StopMotors()
{
	drivetrain->SetSpeeds(0, 0);
}

/**
 * Uses gyro and pid controlled motors to follow a heading.
 * *** CRITICAL: Before using function ResetPID() must be ***
 * *** called(only once) to clear saved variable values.  ***
 */
bool Motors::FollowHeading(float heading_deg, unsigned long desired_time_micros /* = 0UL*/)
{
	if(desired_time_micros > 0)
	{
		if(timer_ == 0) timer_ = micros();
		if(micros() - timer_ > desired_time_micros)
		{
			timer_ = 0;
			return true;
		}
	}
	float diff = heading_deg - GetDegrees();
	if(diff > 180.0f)
		diff -= 360;
	else if(diff < -180.0f)
		diff += 360;
//TODO: Update PID values
	GoUsingPIDControl(0, -diff, 1, 0, 0);
	return false;
}

//Update the angle of the robot based on "Gyrodometry" (http://www-personal.umich.edu/~johannb/Papers/paper63.pdf).
//Basically, it uses the encoders to determine the robot's heading unless the difference between the encoders
//angle and the gyros angle is above a threshold, then it uses the gyro data.
void Motors::UpdateGyrodometry()
{
	//Get time between samples.
	static unsigned long previous_time = 0;
	unsigned long current_time = micros();
	float sample_time_secs = (current_time - previous_time) / 1000000.0f; //time(in seconds) between getting new data
	previous_time = current_time;

	/*--- get current values needed to calculate rate of rotation for gyro and encoders ---*/
	//Get how many ticks passed since last update for encoders
	long delta_left_ticks = drivetrain->left_encoder_ticks_ - drivetrain->prev_left_ticks_;
	long delta_right_ticks = drivetrain->right_encoder_ticks_ - drivetrain->prev_right_ticks_;
	//Save current ticks for next time
	drivetrain->prev_left_ticks_ = drivetrain->left_encoder_ticks_;
	drivetrain->prev_right_ticks_ = drivetrain->right_encoder_ticks_;

	//Read current data for gyro
	gyro_->l3g_gyro_.read();

	/*--- Calibrate those values ---*/
	//Gyro:	
	//get calibrated rate
	float gyro_rate = (static_cast<float>(gyro_->l3g_gyro_.z) - gyro_->calibration.averageBiasZ);
	//TODO: Check if we can subract avg bias AND multiply by scale factor in one step, ie:
	//float gyro_rate = ((float)l3g_gyro_.z - calibration.averageBiasZ) * calibration.scaleFactorZ;
	
	//TODO: CHeck if we need this code, or if it messes the calculations up.
	////If we're 93.75% sure (according to Chebyshev) that this data is caused by normal fluctuations, ignore it.
	//if(abs(rateZ) < 4 * calibration.sigmaZ)
	//{
	//	rateZ = 0.0f;
	//}
	
	//Odometry:
	//Calculate change in mms of both motors and the robot itself
	float delta_left_mms = delta_left_ticks * drivetrain->LEFT_MMS_PER_TICK;
	float delta_right_mms = delta_right_ticks * drivetrain->RIGHT_MMS_PER_TICK;
	//float delta_mms = (delta_left_mms + delta_right_mms) / 2.0;

	//Calculate the change in angle (in radians)
	float delta_theta = (delta_left_mms - delta_right_mms) / drivetrain->WHEELBASE;
	//Get degrees per second
	float odometry_rate = (delta_theta * RADS) / sample_time_secs;

	/*--- Compare rates and update the robot's heading. ---*/
	//Check if the rate between the gyro and odometry differ significantly. If so, then use the gyro
	//to cover changes where the odometry made an error. Otherwise, use the odometry to prevent
	//gyro drift from affecting the angle.
	if(abs(gyro_rate - odometry_rate) > GYRODOMETRY_THRESHOLD)
	{
		gyrodometry_angle_ += gyro_rate * sample_time_secs; 
	}
	else
	{
		gyrodometry_angle_ += delta_theta;
	}
	//Clip the angle to 0~2pi
	gyrodometry_angle_ -= static_cast<int>(gyrodometry_angle_ / 360.0f)*360.0f;

	//TODO: Determine if we need to update our position.
	//Now calculate and accumulate our position in mms
	//Y_pos += delta_mms * cos(theta);
	//X_pos += delta_mms * sin(theta);
}

//Combines the gyro and the encoders (gyrodometry) to get the heading of the robot in degrees.
float Motors::GetDegrees()
{
	//Update both readings and determine best one to use for the angle only when the gyro has fresh data.
	if(gyro_->l3g_gyro_.fresh_data) UpdateGyrodometry(); 
	return gyrodometry_angle_;
}

//Close servos to grab the victim
bool Motors::BiteVictim()
{
	unsigned long curr_time = micros();
	//If timer hasn't been set, set it and start servos closing.
	if(timer_ == 0)
	{
		timer_ = curr_time;
		victim_servo_.write(victim_servo_closed_angle_);
		//right_servo_.write(right_servo_closed_angle_);
	}
	//Check if servos have been closing for long enough
	if(curr_time - timer_ > servo_close_time_)
	{
		timer_ = 0; //reset timer
		return true;
	}
	//time hasn't been long enough
	return false;
}

//Open servos to release the victim
bool Motors::ReleaseVictim()
{
	unsigned long curr_time = micros();
	//If timer hasn't been set, set it and start servos closing.
	if(timer_ == 0)
	{
		timer_ = curr_time;
		victim_servo_.write(victim_servo_open_angle_);
		//right_servo_.write(right_servo_open_angle_);
	}
	//Check if servos have been opening for long enough
	if(curr_time - timer_ > servo_open_time_)
	{
		timer_ = 0; //reset timer
		return true;
	}
	//time hasn't been long enough
	return false;
}