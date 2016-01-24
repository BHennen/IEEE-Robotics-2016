#include "Brain.h"

/**
* Constructor.
*/
Brain::Brain(BrainModules brain_modules, BrainConfig brain_config) : sensor_gap_min_dist_(brain_config.sensor_gap_min_dist),
desired_dist_to_wall_(brain_config.desired_dist_to_wall), front_sensor_stop_dist_(brain_config.front_sensor_stop_dist),
pixy_block_detection_threshold_(brain_config.pixy_block_detection_threshold)
{	
	visual_sensor_ = brain_modules.visual_sensor;					  
	wall_sensors_ = brain_modules.wall_sensors;						 
	motors_ = brain_modules.motors;
	gyro_ = brain_modules.gyro;

	front_detected_ = false;
	reset_pid_ = true;
	last_heading_ = 0.0;

	state_ = RobotState(brain_config.init_direction, brain_config.init_x, brain_config.init_y);
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
	if((flags & StopConditions::FRONT) == StopConditions::FRONT)
	{
		if(visual_sensor_->ReadProximity() < front_sensor_stop_dist_)
		{
			//front sensor got too close.
			Reset();
			return StopConditions::FRONT;
		}
	}

	//check for gap 
	if((flags & StopConditions::GAP) == StopConditions::GAP)
	{
		//Check if front sensor has detected a gap (only once)
		if(!front_detected_ && front_dist > sensor_gap_min_dist_)
		{
			front_detected_ = true; //set flag for rear sensor to start detection
			reset_pid_ = true; //Motors go straight using gyro after gap detected; reset PID
			last_heading_ = gyro_->GetDegrees(); //Save most recent heading so we can continue to go straight using gyro
		}
		//Check if rear sensor has detected a gap (only once)
		if(front_detected_ && rear_dist > sensor_gap_min_dist_)
		{
			Reset();
			return StopConditions::GAP; //Return true to signal we arrived at stop condition
		}
	}

	//check if pixy detects a good block consecutively enough times 
	if((flags & StopConditions::PIXY) == StopConditions::PIXY)
	{
		//Get block & check if it is good
		if(visual_sensor_->IsGoodBlock(visual_sensor_->GetBlock()))
		{
			good_block_count_++;
			if(good_block_count_ >= pixy_block_detection_threshold_)
			{
				//we've detected enough blocks consecutively with the pixy
				Reset();
				return StopConditions::PIXY;
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
		return StopConditions::NONE;
	}

//TODO: Update PID values
	//Follow Wall
	//For now, ignore rear sensor reading and try to maintain the desired distance from the wall using front sensor
	if(dir == RIGHT)
		motors_->GoUsingPIDControl(desired_dist_to_wall_, front_dist, 1, 0, 0);
	else
		motors_->GoUsingPIDControl(front_dist, desired_dist_to_wall_, 1, 0, 0); //Invert desired & current for left wall

	return StopConditions::NONE;
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
	if(!front_detected_ && front_dist < sensor_gap_min_dist_)
	{
		front_detected_ = true; //set flag for rear sensor to start detection
	}
	//Check if rear sensor has detected a wall
	if(front_detected_ && rear_dist < sensor_gap_min_dist_)
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

//Turns the motors 90 deg (using the same function in motors, but allowed to be called from brain class.)
bool Brain::Rotate90(Direction dir)
{
	return motors_->Turn90(dir);
}

//Combine FollowWall and turn functions to go to a position on the board. Returns a state depending on what it encounters
//along the way.
GoAToBState Brain::GoAtoB(Position start_pos, Position end_pos)
{
	GoAToBState curr_state = GOING; //Keep track of the state of the robot.
	static byte step_num = 0; //Keep track of what step we're on.

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
	auto DefaultAction = [this, start_pos, end_pos]() -> GoAToBState
	{
		static byte step_num_2 = 0;
		GoAToBState state = GOING;
		switch(step_num_2)
		{
		case 0:
			//First go to crossroad from start_pos
			state = GoAtoB(start_pos, CROSSROAD);
			if(state == SUCCESS)
			{
				step_num_2++;
			}
			break;
		case 1:
			//Go to end_pos from CROSSROAD
			state = GoAtoB(CROSSROAD, end_pos);
			if(state == SUCCESS)
			{
				step_num_2 = 0;
				motors_->StopMotors();
			}
			break;
		}
		return state;
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
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
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
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now past the wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
	case RED: //Starting at RED (assuming we face TOWARDS the dumping zone)
		switch(end_pos)
		{
		case START: //Ending at START
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
				//Follow left wall until we have reached detected wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
					//Detected wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
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
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
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
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now past the wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default
			curr_state = DefaultAction();
			break;
		}
		break;
	case YELLOW: //Starting at YELLOW (assuming we face TOWARDS the dumping zone)
		switch(end_pos)
		{
		case START: //Ending at START
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
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
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
				//Go straight past wall on the left.
				if(TravelPastWall(LEFT))
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
				//Follow left wall until we have reached detected wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
					//Detected wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		case CROSSROAD: //Ending at CROSSROAD
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow left wall until we have reached the gap in the crossroad.
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now past the wall. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default
			curr_state = DefaultAction();
			break;
		}
		break;
	case CROSSROAD: //Start at CROSSROAD. Depending on where we're going will determine where we're facing.
		switch(end_pos)
		{
		case START:
			break;
		case RED:
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
			break;
		}
		break;
	case CITY_R: //Start at CITY_R (assuming we face TOWARDS the victim)
		switch(end_pos)
		{
		case CROSSROAD: //End at CROSSROAD
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow right wall until we have reached the gap.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
					step_num++;
				}
				break;
			case 3:
				//Go straight past wall on the right.
				if(TravelPastWall(RIGHT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 4:
				//Follow right wall until we have reached the gap in the crossroad.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now past the wall. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
		break;
	case CITY_L: //Start at CITY_L (assuming we face TOWARDS the victim)
		switch(end_pos)
		{
		case CROSSROAD: //End at CROSSROAD
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow right wall until we have reached the gap.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
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
				//Go straight past wall on the left.
				if(TravelPastWall(LEFT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 5:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
		break;
	case MEXICO: //Start at MEXICO (assuming we face TOWARDS the victim)
		switch(end_pos)
		{
		case CROSSROAD: //End at CROSSROAD
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow right wall until we have reached the gap.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
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
				//Go straight past wall on the right.
				if(TravelPastWall(RIGHT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 5:
				//Go straight past another wall on the right.
				if(TravelPastWall(RIGHT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 6:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
		break;
	case USA: //Start at USA (assuming we face TOWARDS the victim)
		switch(end_pos)
		{
		case CROSSROAD: //End at CROSSROAD 
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow left wall until we hit the wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
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
				//Follow left wall until we hit the wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
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
				//Follow left wall until we hit the wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
					step_num++;
				}
				break;
			case 7:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 8:
				//Follow left wall until we find a gap
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
				{
					step_num++;
				}
				break;
			case 9:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 10:
				//Continue past wall to left
				if(TravelPastWall(LEFT))
				{
					step_num++;
				}
				break;
			case 11:
				//Continue past another wall to left
				if(TravelPastWall(LEFT))
				{
					step_num++;
				}
				break;
			case 12:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 13:
				//Continue until we're on the wall to right.
				if(TravelPastWall(RIGHT))
				{
					step_num++;
				}
				break;
			case 14:
				//Follow right wall until we're at the crossroads gap.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now in the gap. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
		break;
	case FRONTIER: //Start at FRONTIER (assuming we face TOWARDS MEXICO)
		switch(end_pos)
		{
		case MEXICO: //End at MEXICO (there was a victim here, grab it!)
			switch(step_num)
			{
			case 0:
				//Go straight past wall on the left.
				if(TravelPastWall(LEFT))
				{
					//Now past the wall.
					step_num++;
				}
				break;
			case 1:
				//Follow left wall and react based on the return value.
				switch(FollowWall(LEFT, StopConditions::PIXY | StopConditions::FRONT))
				{
				case StopConditions::PIXY:
					//Victim found. Mission complete. 
					motors_->StopMotors(); //Stop
					step_num = 0; //Reset step number for next time.
					curr_state = STOP_PIXY; //Signal mission complete (the pixy identified a victim)
					break;
				case StopConditions::FRONT:
					//Pixy didn't detect anything before a wall was found in front. Let calling function deal with it.
					step_num = 0; //Reset step number for next time.
					curr_state = STOP_FRONT;
					break;
				case StopConditions::NONE: //No stop condition, keep going.
					curr_state = GOING;
					break;
				}
				break;
			}
			break;
		case GRASS_S: //End at GRASS_S (there was no victim at Mexico. Assuming we're still facing MEXICO)
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Continue across the long gap until the other wall is found to the right
				if(TravelPastWall(RIGHT))
				{
					step_num++;
				}
				break;
			case 3:
				//Follow right wall until we hit the wall in front.
				if(FollowWall(RIGHT, StopConditions::FRONT) == StopConditions::FRONT)
				{
					step_num++;
				}
				break;
			case 4:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 5:
				//Follow right wall until we hit the wall in front(meaning no GRASS_S victim), or PIXY finds the victim.
				switch(FollowWall(RIGHT, StopConditions::FRONT | StopConditions::PIXY))
				{
				case StopConditions::PIXY:
					//Victim found. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = STOP_PIXY; //Signal mission complete (the pixy identified a victim)
					break;
				case StopConditions::FRONT:
					//Pixy didn't detect anything before a wall was found in front. Let calling function deal with it.
					step_num = 0; //Reset step number for next time.
					curr_state = STOP_FRONT;
					break;
				case StopConditions::NONE: //No stop condition, keep going.
					curr_state = GOING;
					break;
				}
				break;
			}
			break;
		default: //Ending anywhere else, print error. (do not go anywhere but MEXICO and GRASS_S from frontier)
			PrintError();
			curr_state = ERROR;
			break;
		}
		break;
	case GRASS_S: //Start at GRASS_S 
		switch(end_pos)
		{
		case GRASS_N: //End at GRASS_N (assuming we face upwards, facing the wall)
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
				//Follow right wall until PIXY finds the victim 
				//Or, if we somehow miss the victim, until it stops at the next wall.
				switch(FollowWall(RIGHT, StopConditions::FRONT | StopConditions::PIXY))
				{
				case StopConditions::PIXY:
					//Victim found. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = STOP_PIXY; //Signal mission complete (the pixy identified a victim)
					break;
				case StopConditions::FRONT:
					//Pixy didn't detect anything before a wall was found in front. Let calling function deal with it.
					step_num = 0; //Reset step number for next time.
					curr_state = STOP_FRONT;
					break;
				case StopConditions::NONE: //No stop condition, keep going.
					curr_state = GOING;
					break;
				}
				break;
			}
			break;
		case CROSSROAD: //End at CROSSROAD (assuming we face upwards, facing the victim)
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow left wall until we hit the wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
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
				//Follow left wall until we find a gap
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
				{
					step_num++;
				}
				break;
			case 5:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 6:
				//Continue past wall to left
				if(TravelPastWall(LEFT))
				{
					step_num++;
				}
				break;
			case 7:
				//Continue past another wall to left
				if(TravelPastWall(LEFT))
				{
					step_num++;
				}
				break;
			case 8:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 9:
				//Continue until we're on the wall to right.
				if(TravelPastWall(RIGHT))
				{
					step_num++;
				}
				break;
			case 10:
				//Follow right wall until we're at the crossroads gap.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now in the gap. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
		break;
	case GRASS_N: //Start at GRASS_N (assuming we face left, facing the victim)
		switch(end_pos)
		{
		case CROSSROAD: //End at CROSSROAD 
			switch(step_num)
			{
			case 0:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 1:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90 (total 180), go to next step.
					step_num++;
				}
				break;
			case 2:
				//Follow left wall until we hit the wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
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
				//Follow left wall until we hit the wall in front.
				if(FollowWall(LEFT, StopConditions::FRONT) == StopConditions::FRONT)
				{
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
				//Follow left wall until we find a gap
				if(FollowWall(LEFT, StopConditions::GAP) == StopConditions::GAP)
				{
					step_num++;
				}
				break;
			case 7:
				//Turn counter clockwise
				if(motors_->Turn90(LEFT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 8:
				//Continue past wall to left
				if(TravelPastWall(LEFT))
				{
					step_num++;
				}
				break;
			case 9:
				//Continue past another wall to left
				if(TravelPastWall(LEFT))
				{
					step_num++;
				}
				break;
			case 10:
				//Turn clockwise
				if(motors_->Turn90(RIGHT))
				{
					//rotated 90, go to next step.
					step_num++;
				}
				break;
			case 11:
				//Continue until we're on the wall to right.
				if(TravelPastWall(RIGHT))
				{
					step_num++;
				}
				break;
			case 12:
				//Follow right wall until we're at the crossroads gap.
				if(FollowWall(RIGHT, StopConditions::GAP) == StopConditions::GAP)
				{
					//Now in the gap. Mission complete.
					step_num = 0; //Reset step number for next time.
					curr_state = SUCCESS; //Signal mission complete
				}
				break;
			}
			break;
		default: //Ending anywhere else, use default.
			curr_state = DefaultAction();
			break;
		}
		break;
	default:
		//Shouldn't end up in here. Every start position on the board should go SOMEwhere.
		PrintError();
		curr_state = ERROR;
		break;
	}
	return curr_state;
}

