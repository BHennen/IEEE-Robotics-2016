#include "Brain.h"
#include "Brain.h"

/**
* Constructor.
*/
Brain::Brain(BrainModules brain_modules, BrainConfig brain_config)
{
	visual_sensor_ = brain_modules.visual_sensor;
	wall_sensors_ = brain_modules.wall_sensors;
	motors_ = brain_modules.motors;
	gyro_ = brain_modules.gyro;

	config = brain_config;

	front_detected_ = false;
	reset_pid_ = true;
	last_heading_ = 0.0;
}

//Destructor
Brain::~Brain()
{

}

/**
* Use sensors to follow a wall to the [direction] of the robot until it meets a stop condition, then stops motors.
* Returns whichever condition it stopped on when any of the stop conditions set by flags are met, or NONE
* if it doesn't encounter a stop condition.
* stop conditions :
*		FRONT -- Check front IR sensor and return FRONT when it it close to wall in front.
*		GAP -- Check if gap was found in the [direction]. Return GAP when both sensors have detected the gap.
*		PIXY -- Check if pixy has detected enough consecutive good blocks. Return PIXY when it has detected enough blocks.
*		NONE -- Follow wall indefinitely.
*
* To call this: FollowWall(LEFT, GAP | PIXY); <-- This will follow left wall and stop when a gap is detected or pixy sees a victim
*/
StopConditions Brain::FollowWall(Direction dir, StopConditions flags)
{
	// Record sensor readings ////////////////
	float front_dist;
	float rear_dist;
	if(dir == LEFT)
	{
		front_dist = wall_sensors_->ReadSensor(FRONT_LEFT);
		rear_dist = wall_sensors_->ReadSensor(REAR_LEFT);
	}
	else
	{
		front_dist = wall_sensors_->ReadSensor(FRONT_RIGHT);
		rear_dist = wall_sensors_->ReadSensor(REAR_RIGHT);
	}

	// Check stop conditions ////////////////////////

	//Lambda function that is called after a stop condition has been found to reset flags & stop motors
	auto Reset = [this]()
	{
		this->motors_->StopMotors(); //Stop 
		this->front_detected_ = false;  //reset front_detected_ flag for next time
		this->reset_pid_ = true;     //reset PID for next time
		good_block_count_ = 0;       //reset pixy block counts for next time
	};

	//check if front is too close
	if(flags & FRONT)
	{
		if(visual_sensor_->ReadProximity() < config.front_sensor_stop_dist)
		{
			//front sensor got too close.
			Reset();
			return FRONT;
		}
	}

	//check for gap 
	if(flags & GAP)
	{
		//Check if front sensor has detected a gap (only once)
		if(!front_detected_ && front_dist > config.sensor_gap_min_dist)
		{
			front_detected_ = true; //set flag for rear sensor to start detection
			reset_pid_ = true; //Motors go straight using gyro after gap detected; reset PID
			last_heading_ = gyro_->GetDegrees(); //Save most recent heading so we can continue to go straight using gyro
		}
		//Check if rear sensor has detected a gap (only once)
		if(front_detected_ && rear_dist > config.sensor_gap_min_dist)
		{
			Reset();
			return GAP; //Return true to signal we arrived at stop condition
		}
	}

	//check if pixy detects a good block consecutively enough times 
	if(flags & PIXY)
	{
		//Get block & check if it is good
		if(visual_sensor_->IsGoodBlock(visual_sensor_->GetBlock()))
		{
			good_block_count_++;
			if(good_block_count_ >= config.pixy_block_detection_threshold)
			{
				//we've detected enough blocks consecutively with the pixy
				Reset();
				return PIXY;
			}
		}
		else
		{
			good_block_count_ = 0; //reset block count
		}
	}

	// Power Motors //////////////////////////////

	//Reset PID if we haven't already (or we need to again)
	if(reset_pid_)
	{
		motors_->ResetPID();
		reset_pid_ = false;
	}

	//If gap was started, go straight using gyro until gap detected by rear sensor
	if(front_detected_)
	{
		motors_->FollowHeading(last_heading_);
		return NONE;
	}

//TODO: Update PID values
	//Follow Wall
	//For now, ignore rear sensor reading and try to maintain the desired distance from the wall using front sensor
	if(dir == RIGHT)
		motors_->GoUsingPIDControl(config.desired_dist_to_wall, front_dist, 1, 0, 0);
	else
		motors_->GoUsingPIDControl(front_dist, config.desired_dist_to_wall, 1, 0, 0); //Invert desired & current for left wall

	return NONE;
}


//Use pixy and other sensor to go to victim. Return true when it has stopped in the right position.
bool Brain::GoToVictim()
{
	return false;
}

//Go straight until a wall has been detected by both the front and rear sensors and then stops.
//Return true when it has done so, otherwise return false.
bool Brain::TravelPastWall(Direction dir)
{
	// Record sensor readings ////////////////
	float front_dist;
	float rear_dist;
	if(dir == LEFT)
	{
		front_dist = wall_sensors_->ReadSensor(FRONT_LEFT);
		rear_dist = wall_sensors_->ReadSensor(REAR_LEFT);
	}
	else
	{
		front_dist = wall_sensors_->ReadSensor(FRONT_RIGHT);
		rear_dist = wall_sensors_->ReadSensor(REAR_RIGHT);
	}

	if(reset_pid_) last_heading_ = gyro_->GetDegrees(); //Update heading before we begin

	//Check if front sensor has detected a wall (only once)
	if(!front_detected_ && front_dist < config.sensor_gap_min_dist)
	{
		front_detected_ = true; //set flag for rear sensor to start detection
	}
	//Check if rear sensor has detected a wall
	if(front_detected_ && rear_dist < config.sensor_gap_min_dist)
	{
		motors_->StopMotors(); //Stop 
		front_detected_ = false;  //reset front_detected_ flag for next time
		reset_pid_ = true;     //reset PID for next time
		return true; //Return true to signal both sensors detected a wall.
	}

	// Power Motors //////////////////////////////

	//Reset PID if before we begin
	if(reset_pid_)
	{
		motors_->ResetPID();
		reset_pid_ = false;
	}

	//Follow the heading that the robot had when this function was initially called.
	motors_->FollowHeading(last_heading_);

	return false;
}

//Combine FollowWall and turn functions to go to a position on the board. Returns true when it is there.
bool Brain::GoAtoB(Position start_pos, Position end_pos)
{
	static byte step_num = 0;

	//Print that the directions from start_pos to end_pos are not specified in the code.
	auto PrintError = [start_pos, end_pos]()
	{
		Serial.print(F("Error, start_pos = "));
		Serial.print(start_pos);
		Serial.print(F(" and end_pos = "));
		Serial.print(end_pos);
		Serial.println(F(" do not have code in the GoAtoB function."));
	};
	
	//By default, go from start_pos to CROSSROAD then from CROSSROAD to end_pos
	//Stop and return true when we get there.
	auto DefaultAction = [this, start_pos, end_pos]() -> bool
	{
		static byte step_num_2 = 0;
		switch(step_num_2)
		{
		case 0:
			//First go to crossroad
			if(GoAtoB(start_pos, CROSSROAD))
			{
				step_num_2++;
			}
			break;
		case 1:
			//From crossroad go to end_pos
			if(GoAtoB(CROSSROAD, end_pos))
			{
				step_num_2 = 0;
				motors_->StopMotors();
				return true;
			}
			break;
		}
		return false;
	};

	//Take Action based on starting location
	switch(start_pos)
	{
	case START: //Starting at START
		switch(end_pos)
		{
		case CROSSROAD: //End at CROSSROAD
			switch(step_num)
			{
			case 0:
				//Go from start to space in front of start.
				if(FollowWall(LEFT, GAP))
				{
					//gap detected; go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 2:
				//Go straight past wall on the right.
				if(TravelPastWall(RIGHT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 3:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 4:
				//Follow left wall until we have reached the gap in the crossroad.
				if(FollowWall(LEFT, GAP) == GAP)
				{
					//Now past the wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					return true; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			return DefaultAction();
			break;
		}
	case RED: //Starting at RED (assuming we face TOWARDS the dumping zone)
		switch(end_pos)
		{
		case CROSSROAD: //Ending at CROSSROAD
			switch(step_num)
			{
			case 0:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow right wall until we have reached the gap.
				if(FollowWall(RIGHT, GAP) == GAP)
				{
					step_num++;
				}
			case 3:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 4:
				//Go straight past wall on the right.
				if(TravelPastWall(RIGHT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 5:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 6:
				//Follow left wall until we have reached the gap in the crossroad.
				if(FollowWall(LEFT, GAP) == GAP)
				{
					//Now past the wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					return true; //Signal mission complete
				}
				break;
			}
		default: //Ending anywhere else, use default
			return DefaultAction();
			break;
		}
		break;
	case YELLOW:
		break;
	case CROSSROAD:
		break;
	case CITY_R:
		break;
	case CITY_L:
		break;
	case MEXICO:
		break;
	case USA:
		break;
	case FRONTIER:
		break;
	case GRASS_S:
		break;
	case GRASS_N:
		break;
	default:
		//Shouldn't end up in here. Every start position on the board should go SOMEwhere.
		Serial.print(F("Start position unaccounted for:"));
		Serial.println(start_pos);
		break;
	}
	return false;
}

