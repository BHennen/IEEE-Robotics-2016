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

	PID_out_max = 255 - drive_power_;
	PID_out_min = -255 + static_cast<short>(drive_power_);

	victim_servo_.attach(motor_config.victim_servo_pin, 800, 2200);

	victim_servo_closed_angle_ = motor_config.victim_servo_closed_angle;
	victim_servo_open_angle_ = motor_config.victim_servo_open_angle;

	servo_close_time_ = motor_config.servo_close_time;
	servo_open_time_ = motor_config.servo_open_time;

	GYRODOMETRY_THRESHOLD = motor_config.GYRODOMETRY_THRESHOLD;

	PID_sample_time_ = motor_config.PID_sample_time;

	//Enable timer 4 timer
	//TODO: Make timer selectable; i.e. which pin is being used for the PID controller timer interrupt.
	//Currently, use Pin 6, timer 4A
	TCCR4A = 0; //Normal operation
	TCCR4B = bit(WGM42) | bit(CS41); //Set CTC mode, scale to clock / 8 ( = 2 microseconds)
	OCR4A = 49999; //Set the compare register to (49999 + 1) microseconds = 25 milliseconds; 40Hz update for PID

	victim_servo_.write(victim_servo_closed_angle_);
}

//Destructor
Motors::~Motors()
{

}

//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
bool Motors::Turn90(Direction dir)
{
	//Code that uses the gyrodometery
	float current_degrees = gyro_->GetDegrees();
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

//Sets the constants for the PID controller as well as the desired sample time, then starts it.
void Motors::StartPID(float set_point, float input, bool reverse, bool inverse, float kp, float ki, float kd, unsigned long sample_time)
{
	//Update input value
	this->input = input;

	//Update PID tunings
	if(!pid_running)
	{
		//Set tunings
		this->set_point = set_point;
		this->reverse = reverse;
		this->inverse = inverse;

		this->power = reverse ? -static_cast<short>(drive_power_) : drive_power_;

		//PID_sample_time_ = sample_time; //TODO: Make sample time adjustable
		float sample_time_seconds = static_cast<float>(PID_sample_time_) / 1000000.0;
		this->kp = kp;
		this->ki = ki * sample_time_seconds;
		this->kd = kd / sample_time_seconds;

		ResetPID(input); //Reset the PID values

		//Start PID by enabling the PID timer interrupt
		TIMSK4 |= bit(OCIE4A); //Enable timer interrupt on pin 6 (timer 4A)
	}
	//else running; do nothing
}

//Sets the constants for the PID controller.
void Motors::StartPID(float set_point, float input, bool reverse, bool inverse, float kp, float ki, float kd)
{
	StartPID(set_point, input, reverse, inverse, kp, ki, kd, PID_sample_time_);
}

//Turn off timer interrupt for PID and signal that the PID has stopped running. Also stop motors.
void Motors::StopPID()
{
	//Turn off timer interrupt
	TIMSK4 &= ~bit(OCIE4A); //Stop timer interrupt on pin 6 (timer 4A)
	//Signal PID is not running
	pid_running = false;
	//Stop motors
	StopMotors();
	set_init_values = true;
}

//Resets the saved values for the PID controller of the motors.
//Takes a float input so that when it calculates the derivative there is no output spike.
void Motors::ResetPID(float input)
{
	integral_ = 0.0;
	previous_input_ = input;
}

/**
 * Uses PID control to go in a direction. Given an input and a set_point, updates PID values and
 * powers the motors to try to achieve the set_point. Input should be configured such that
 * set_point < input means the robot will try to turn left, unless it is inverted.
 * Based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
void Motors::RunPID()
{
	//Determine error; how far off the input is from desired set point
	float error = set_point - input;

	//Determine integral; sum of all errors. 
	integral_ += ki * error;
	//Also, constrain it so it is within PWM limits.
	integral_ = constrain(integral_, PID_out_min, PID_out_max);

	//Determine derivative on measurement; rate of change of the inputs
	float derivative = input - previous_input_;

	//Determine output, invert if necessary, and constrain it within PWM limits
	short output = static_cast<short>(kp * error + integral_ - kd * derivative);
	if(inverse) output *= -1;
	output = constrain(output, PID_out_min, PID_out_max);

	//Go with the adjusted power values
	drivetrain->SetSpeeds(power + output, power - output);

	//Save current input
	previous_input_ = input;

	pid_running = true;
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
bool Motors::FollowHeading(float heading_deg, unsigned long desired_time_micros /* = 0UL*/, float desired_distance_mm /*= 0.0*/, bool reverse /*= false*/)
{
	static bool set_init_values = true;
	//Check if we need to stop after a certain time has passed.
	if(desired_time_micros > 0)
	{
		//Set timer if it is == 0
		if(timer_ == 0) timer_ = micros();

		//Check if desired time has passed, if so reset initial values for next time and return true. 
		if(micros() - timer_ > desired_time_micros)
		{
			timer_ = 0;
			set_init_values = true;
			return true;
		}
	}

	//Check if we need to go a desired distance.
	if(desired_distance_mm > 0)
	{
		//Set initial values		
		static float init_X_mms;
		static float init_Y_mms;
		if(set_init_values)
		{
			init_X_mms = GetX();
			init_Y_mms = GetY();
			set_init_values = false;
		}

		//Check if the distance has been reached
		float delta_X_mms = GetX() - init_X_mms;
		float delta_Y_mms = GetY() - init_Y_mms;
		float delta_mms = sqrt(pow(delta_X_mms, 2) + pow(delta_Y_mms, 2));
		if(delta_mms >= desired_distance_mm)
		{
			//If so, reset init values for next time and return true
			timer_ = 0;
			set_init_values = true;
			return true;
		}
	}

	//Lambda to get difference between heading and current degrees
	float diff = gyro_->GetDegrees() - heading_deg; // > 0 means need to turn left
	if(diff > 180.0f)
		diff -= 360;
	else if(diff < -180.0f)
		diff += 360;

	StartPID(0.0, diff, reverse, false, 1.0, 0.0, 0.0); //TODO: Update PID values
	return false;
}

/**
 * Uses encoders and PID control to go straight (ignoring the gyro).
 * *** CRITICAL: Before using function ResetPID() must be ***
 * *** called(only once) to clear saved variable values.  ***
 */
bool Motors::GoStraight(unsigned long desired_time_micros/* = 0UL*/, float desired_distance_mm /*= 0.0*/, bool reverse /*= false*/)
{
	//Set initial values		
	static float init_left_mms;
	static float init_right_mms;
	if(set_init_values)
	{
		init_left_mms = drivetrain->left_encoder_ticks_ * drivetrain->LEFT_MMS_PER_TICK;
		init_right_mms = drivetrain->right_encoder_ticks_ * drivetrain->RIGHT_MMS_PER_TICK;
		set_init_values = false;
	}

	//Check if we need to stop after a certain time has passed.
	if(desired_time_micros > 0)
	{
		//Set timer if it is == 0
		if(timer_ == 0) timer_ = micros();

		//Check if desired time has passed, if so reset initial values for next time and return true. 
		if(micros() - timer_ > desired_time_micros)
		{
			timer_ = 0;
			set_init_values = true;
			return true;
		}
	}

	float delta_left_mms = drivetrain->left_encoder_ticks_ * drivetrain->LEFT_MMS_PER_TICK - init_left_mms;
	float delta_right_mms = drivetrain->right_encoder_ticks_ * drivetrain->RIGHT_MMS_PER_TICK - init_right_mms;

	//Check if we need to go a desired distance.
	if(desired_distance_mm > 0)
	{
		//Check if the distance has been reached
		
		float delta_mms = (delta_left_mms + delta_right_mms) / 2.0;
		if(abs(delta_mms) >= desired_distance_mm)
		{
			//If so, reset init values for next time and return true
			timer_ = 0;
			set_init_values = true;
			return true;
		}
	}

	//Go forward using PID control
	float diff = delta_left_mms - delta_right_mms; // >0 means need to turn left

	StartPID(0.0, diff, reverse, false, 3.0, 1.0, 0.2); //TODO: Update PID values
	return false;
}

//Uses the encoders to rotate to a certain angle.
bool Motors::Rotate(Direction dir, uint16_t angle /*= 90*/, bool sweep/* = false*/)
{
	static float init_left_mms;
	static float init_right_mms;

	float left_mms = drivetrain->left_encoder_ticks_ * drivetrain->LEFT_MMS_PER_TICK;
	float right_mms = drivetrain->right_encoder_ticks_ * drivetrain->RIGHT_MMS_PER_TICK;

	//Calculate the desired degrees
	if(!rotating_)
	{
		//Set the robots desired degrees based on current degrees
		desired_degrees_ = (dir == RIGHT) ? static_cast<float>(angle) : -static_cast<float>(angle);
		//Set initial mms of left and right wheels
		init_left_mms = left_mms;
		init_right_mms = right_mms;

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

	//Calculate the change in angle since we started.
	float delta_left_mms = left_mms - init_left_mms;
	float delta_right_mms = right_mms - init_right_mms;
	float delta_theta = (delta_left_mms - delta_right_mms) / drivetrain->WHEELBASE / 2;
	float diff = desired_degrees_ - delta_theta * RAD_TO_DEG;
	if(diff > 180.0f)
		diff -= 360;
	else if(diff < -180.0f)
		diff += 360;
	if(abs(diff) < 1.0)
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
		if(sweep)
		{
			if(turn_direction == RIGHT)
			{
				//sweep to the right, keeping right wheel stationary
				drivetrain->SetSpeeds(power, 0);
			}
			else
			{
				//sweep to the left, keeping left wheel stationary
				drivetrain->SetSpeeds(0, power);
			}
		}
		else
		{
			TurnStationary(drive_power_, turn_direction);
		}

		rotating_ = true;
		return false;
	}

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
	float delta_mms = (delta_left_mms + delta_right_mms) / 2.0;

	//Calculate the change in angle (in radians)
	float delta_theta = (delta_left_mms - delta_right_mms) / drivetrain->WHEELBASE;
	//Get degrees per second
	float odometry_rate = (delta_theta * RAD_TO_DEG) / sample_time_secs;

	/*--- Compare rates and update the robot's heading. ---*/
	//Check if the rate between the gyro and odometry differ significantly. If so, then use the gyro
	//to cover changes where the odometry made an error. Otherwise, use the odometry to prevent
	//gyro drift from affecting the angle.
	//if(abs(gyro_rate - odometry_rate) > GYRODOMETRY_THRESHOLD)
	//{
	//	gyrodometry_angle_ += gyro_rate * sample_time_secs;
	//}
	//else
	//{
	gyrodometry_angle_ += delta_theta;
//}
//Clip the angle to 0~360 deg
	gyrodometry_angle_ -= static_cast<int>(gyrodometry_angle_ / 360.0f)*360.0f;

	//Now calculate and accumulate our position in mms
	Y_pos += delta_mms * cos(gyrodometry_angle_ * DEG_TO_RAD);
	X_pos += delta_mms * sin(gyrodometry_angle_ * DEG_TO_RAD);
}

//Combines the gyro and the encoders (gyrodometry) to get the heading of the robot in degrees.
float Motors::GetDegrees()
{
	//Update both readings and determine best one to use for the angle only when the gyro has fresh data.
	if(gyro_->l3g_gyro_.fresh_data) UpdateGyrodometry();
	Serial.println(gyrodometry_angle_);
	return gyrodometry_angle_;
}

//Get X position of robot in mm, based on encoders and the gyro.
float Motors::GetX()
{
	//Update both readings and determine best one to use for the angle only when the gyro has fresh data.
	if(gyro_->l3g_gyro_.fresh_data) UpdateGyrodometry();
	return X_pos;
}

//Get Y position of robot in mm, based on encoders and the gyro.
float Motors::GetY()
{
	//Update both readings and determine best one to use for the angle only when the gyro has fresh data.
	if(gyro_->l3g_gyro_.fresh_data) UpdateGyrodometry();
	return Y_pos;
}

//Close servos to grab the victim
bool Motors::BiteVictim()
{
	unsigned long curr_time = micros();
	//If timer hasn't been set, set it and start servos closing.
	if(timer_ == 0)
	{
		timer_ = curr_time;
		//right_servo_.write(right_servo_closed_angle_);
	}
	//Check if servos have been closing for long enough
	if(curr_time - timer_ > servo_close_time_)
	{
		timer_ = 0; //reset timer
		return true;
	}
	victim_servo_.write(victim_servo_closed_angle_);
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
	}
	//Check if servos have been opening for long enough
	if(curr_time - timer_ > servo_open_time_)
	{
		timer_ = 0; //reset timer
		return true;
	}
	victim_servo_.write(victim_servo_open_angle_);
	//time hasn't been long enough
	return false;
}

bool Motors::CalibrateGyro()
{
	//setup variables used in the calibration
	enum CalibrationStates
	{
		CalculateBias,
		WaitForInput,
		Rotate,
		MeasureScaleFactor,
		PrintDirections,
		CheckResults
	};
	const float ROTATION_ANGLE = 360.0f; //How far to rotate in calibration procedure
	const float READING_TO_DPS = 0.00875f; //Number to convert raw data to degrees
	static float STOP_RATE; //How slow to be considered stopped
	const unsigned long averagingTime = 5000000UL; //Optimal time based on allan variance is 5 sec

	static float scaleFactor = 0.0f;
	static float averageBiasZ = 0.0f;
	static float sigmaZ = 0.0f;
	static float angleZ = 0.0f;
	static float maxAngleZ = 0.0f;
	static unsigned int numSamples = 0;
	static float M2 = 0.0f;
	static unsigned long timer = 0;
	static bool printResults = true;
	static bool measureScaleFactor = true;
	static bool hasRotated = false;
	float rateZ = 0.0;
	static CalibrationStates current_state = CalculateBias;

	unsigned long current_time = micros(); //read current time at every loop
	static unsigned long prev_time = current_time;
	unsigned long sampleTime = current_time - prev_time;
	bool data_avail = gyro_->l3g_gyro_.read();
	prev_time = current_time;
	switch(current_state)
	{
	case CalculateBias:
		//Check if we have calculated bias for the desired amount of time.
		if(timer == 0)
		{
			Serial.println(F("Calculating Bias"));
			timer = current_time;
		}
		if(current_time - timer < averagingTime)
		{
			//Keep a running average of current value of the gyro
			if(data_avail)
			{
				numSamples++;
				float delta = (float)gyro_->l3g_gyro_.z - averageBiasZ;
				averageBiasZ += delta / numSamples;
				M2 += delta*((float)gyro_->l3g_gyro_.z - averageBiasZ);
			}
		}
		else //Calculated for long enough. Print results
		{
			//calculate sigmaZ, the standard deviation of the Z values
			float variance = M2 / (numSamples - 1);
			sigmaZ = sqrt(variance);

			STOP_RATE = sigmaZ / 2;
			Serial.println(STOP_RATE);
			//Print results
			Serial.println(F("Average Bias:"));
			Serial.print(F("Z= "));
			Serial.println(averageBiasZ);

			Serial.println(F("Sigma:"));
			Serial.print(F("Z= "));
			Serial.println(sigmaZ);

			//reset timer and change state
			timer = 0;
			current_state = PrintDirections;
		}
		break;
	case PrintDirections:
		Serial.print(F("Type into serial anything to start the motors turning. Once it reaches "));
		Serial.print(ROTATION_ANGLE);
		Serial.println(F(" degrees, type something else into the terminal to stop it."));
		current_state = WaitForInput;
		break;
	case WaitForInput:
		//wait for any input
		if(Serial.available() > 0)
		{
			//clear the buffer
			while(Serial.read() > 0);
			//change state to run the motors
			prev_time = micros();
			TurnStationary(40, RIGHT); //go slowly clockwise
			current_state = Rotate;
		}
		break;
	case Rotate: //Rotate & update angle while measuring 
		if(Serial.available() > 0) //check if user wants to stop rotating
		{
			StopMotors();
			current_state = MeasureScaleFactor;
			return false;
		}
		if(timer == 0) timer = current_time; //Reset timer
		if(!data_avail) return false;
		//measure current rate of rotation
		rateZ = (float)gyro_->l3g_gyro_.z - averageBiasZ;
		//Serial.println(rateZ);

		//find angle
		angleZ += (rateZ * (sampleTime / 1000000.0f)); //divide by 1000000.0(convert to sec)

													   //Find max angle
		if(angleZ > maxAngleZ) maxAngleZ = angleZ;
		Serial.println(maxAngleZ); //Don't know why, but need this print statement to calibrate.

		break;
	case MeasureScaleFactor:

		scaleFactor = ROTATION_ANGLE / maxAngleZ;
		Serial.println(F("Done!"));
		Serial.print(F("Max Angle = "));
		Serial.println(maxAngleZ);
		Serial.print(F("Scale Factor = "));
		Serial.println(scaleFactor);

		//Update calibration data
		CalibrationData calibration;
		calibration.averageBiasZ = averageBiasZ;
		calibration.scaleFactorZ = scaleFactor;
		calibration.sigmaZ = sigmaZ;
		//Write calibration data to eeprom
		EEPROM.put(0, calibration);

		//Print out the values put into the eeprom for verification:
		CalibrationData stored_data;
		EEPROM.get(0, stored_data);
		float storedBias = stored_data.averageBiasZ;
		float storedSigma = stored_data.sigmaZ;
		float storedScaleFactor = stored_data.scaleFactorZ;

		Serial.println(F("Printing calculated and stored values..."));
		Serial.print(F("BiasZ = "));
		Serial.print(averageBiasZ);
		Serial.print(F("\tStored = "));
		Serial.println(storedBias);

		Serial.print(F("sigmaZ = "));
		Serial.print(sigmaZ);
		Serial.print(F("\tStored = "));
		Serial.println(storedSigma);

		Serial.print(F("scaleFactorZ = "));
		Serial.print(scaleFactor);
		Serial.print(F("\tStored = "));
		Serial.println(storedScaleFactor);

		return true; //Done calibrating

		break;
	}
	return false;
}