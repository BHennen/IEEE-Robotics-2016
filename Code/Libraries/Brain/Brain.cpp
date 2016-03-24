#include "Brain.h"

//Convert byte_action_list to vector::<Action*> and return it
ActionList Brain::ByteActionListConverter(byte_action_list a_star_results)
{
	ActionList action_list;
	RobotState curr_state = this->robot_state_;
	//loop through list of byte_actions
	for(byte_action action : a_star_results)
	{
		//get program number
		word prog = action.Test(2) * 4 + action.Test(1) * 2 + action.Test(0) * 1;

		//Get direction
		Direction dir;
		if(action.Test(3))
		{
			dir = RIGHT;
		}
		else
		{
			dir = LEFT;
		}
		//Make action based on program and arguments
		Successor temp_succ;
		Action* gen_action;
		if(prog == 0)
		{
			//rotate
			if(SearchAlgorithm::GenerateRotateSuccessor(curr_state, this->board_state_, dir, temp_succ))
			{
				curr_state = temp_succ.state;
				gen_action = new Action(this, temp_succ.state, dir, StopConditions::NONE, StopConditions::NONE, prog);
				action_list.AddAction(gen_action);
			}
			else
			{
				//ERROR: no successor found??? not supposed to happen here
			}
		}
		else if(prog == 1)
		{
			//tpw
			if(SearchAlgorithm::GenerateTravelPastWallSuccessor(curr_state, this->board_state_, dir, temp_succ))
			{
				curr_state = temp_succ.state;
				gen_action = new Action(this, temp_succ.state, dir, StopConditions::NONE, StopConditions::NONE, prog);
				action_list.AddAction(gen_action);
			}
			else
			{
				//ERROR: no successor found??? not supposed to happen here
			}
		}
		else if(prog == 2)
		{
			//follow wall			
			word err = action.Test(11) * 4 + action.Test(10) * 2 + action.Test(9) * 1;
			word suc = action.Test(8) * 4 + action.Test(7) * 2 + action.Test(6) * 1;

			StopConditions err_flags = static_cast<StopConditions>(err);
			StopConditions suc_flags = static_cast<StopConditions>(suc);

			if(SearchAlgorithm::GenerateFollowWallSuccessor(curr_state, this->board_state_, dir, temp_succ))
			{
				curr_state = temp_succ.state;
				gen_action = new Action(this, temp_succ.state, dir, suc_flags, err_flags, prog);
				action_list.AddAction(gen_action);
			}
			else
			{
				//ERROR: no successor found??? not supposed to happen here
			}
		}
		else if(prog == 3)
		{
			//gotovictim
			if(SearchAlgorithm::GenerateGoToVictimSuccessor(curr_state, this->board_state_, temp_succ))
			{
				curr_state = temp_succ.state;
				gen_action = new Action(this, temp_succ.state, dir, StopConditions::NONE, StopConditions::NONE, prog);
				action_list.AddAction(gen_action);
			}
			else
			{
				//ERROR: no successor found??? not supposed to happen here
			}
		}
	}
	return action_list;
}

/**
* Constructor.
*/
Brain::Brain(BrainModules brain_modules, BrainConfig brain_config) : sensor_gap_min_dist_(brain_config.sensor_gap_min_dist),
desired_dist_to_wall_(brain_config.desired_dist_to_wall), front_sensor_stop_dist_(brain_config.front_sensor_stop_dist),
pixy_block_detection_threshold_(brain_config.pixy_block_detection_threshold), squaring_diff_threshold_(brain_config.squaring_diff_threshold)
{
	visual_sensor_ = brain_modules.visual_sensor;
	wall_sensors_ = brain_modules.wall_sensors;
	motors_ = brain_modules.motors;
	gyro_ = brain_modules.gyro;

	front_detected_ = false;
	last_heading_ = 0.0;

	clearing_time_ = brain_config.clearing_time;
	squaring_offset_ = brain_config.squaring_offset;
	done_moving = false;
	has_victim = false;
	num_victims = 0;
	victim_sig = 0;
	robot_state_ = RobotState(brain_config.init_direction, brain_config.init_x, brain_config.init_y);
	board_state_ = BoardState(brain_config.init_board_state);
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
	static bool has_been_squared = false;

	//Make sure we're sqaured to the wall
	if(!has_been_squared)
	{
		if(SquareToWall(dir)) has_been_squared = true;
		return StopConditions::NONE;
	}

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
		has_been_squared = false; //Make sure we square up to the wall the next time
		this->front_detected_ = false;  //reset front_detected_ flag for next time
		this->rear_detected_ = false;
		motors_->StopPID();    //stop PID for this run
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
			motors_->StopPID(); //Motors go straight using gyro after gap detected; stop PID for wall following
		}
		//Check if rear sensor has detected a gap (only once)
		if(!rear_detected_ && front_detected_ && rear_dist > sensor_gap_min_dist_)
		{
			rear_detected_ = true; //Set flag to signal we've passed the gap		
		}
		//Go forward for a little bit so we clear our ass from the wall.
		if(rear_detected_)
		{
			Reset();
			return StopConditions::GAP; //Return true to signal we arrived at stop condition
			//if(motors_->GoStraight(clearing_time_))
			//{
			//	Reset();
			//	return StopConditions::GAP; //Return true to signal we arrived at stop condition
			//}
			//else
			//{
			//	return StopConditions::NONE;
			//}
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

	//If gap was started, go straight using gyro until gap detected by rear sensor
	if(front_detected_)
	{
		motors_->GoStraight();
		return StopConditions::NONE;
	}

	//Follow Wall
	//Tune PID controller
	bool inverted;
	if(dir == RIGHT)
	{
		inverted = false; //invert PID function for right wall following
	}
	else
	{
		inverted = true;
	}

	//For now, ignore rear sensor reading and try to maintain the desired distance from the wall using front sensor
	int error = desired_dist_to_wall_ - front_dist;
	motors_->StartPID(0, error, false, inverted, 4.0, 0.5, 1.5); //TODO: Update PID values

	return StopConditions::NONE;
}


//Use pixy and other sensor to go to victim. Return true when it has stopped in the right position.
//TODO: Maybe set a timer, so if we miss victim we reverse and try again?
//TODO: Maybe once we're really close, make sure we're perfectly aligned with the victim by just rotating
//		in place, and once that's done then we move forward.
bool Brain::GoToVictim()
{
	//Open the jaw to prep for grabbing the victim
	motors_->ReleaseVictim();

	//Once the victim is in the cutout area, success!
	if(visual_sensor_->HasVictim())
	{
		victim_sig = visual_sensor_->GetBlockSignature(true);//record the victim signature and reset counts
		motors_->StopPID(); //Stop PID for this use
		return true;
	}

	//get victim and make sure it's good to go to (if not, return false)
	Block victim = visual_sensor_->GetBlock();
	if(!visual_sensor_->IsGoodBlock(victim)) return false;

	//Go using PID, keeping the victim.x aligned with the center of the pixy's view.
	int error;
	//Check to see if we're close enough to go straight
	if(victim.y + victim.height / 2 >= 190)
	{
		error = 0; //we're close; just go straight
	}
	else
	{
		error = visual_sensor_->GetCenter() - victim.x; //get how far off the victim is from the center
	}
	motors_->StartPID(0, error, false, false, 1.0, 0.0, 0.0); //TODO: Update PID values	

	//Default, return false until victim inside cutout.
	return false;
}

//Go straight until a wall has been detected by both the front and rear sensors and then stops.
//Return true when it has done so, otherwise return false.
bool Brain::TravelPastWall(Direction dir)
{
	// Record sensor readings ////////////////
	static uint16_t num_recordings = 0;
	if(num_recordings < 10) //set dist to average of first 10 values seen
	{
		num_recordings++;
		if(dir == LEFT)
		{
			front_dist += (wall_sensors_->ReadSensor(FRONT_LEFT) - front_dist) / num_recordings;
			rear_dist += (wall_sensors_->ReadSensor(REAR_LEFT) - rear_dist) / num_recordings;
		}
		else
		{
			front_dist += (wall_sensors_->ReadSensor(FRONT_RIGHT) - front_dist) / num_recordings;
			rear_dist += (wall_sensors_->ReadSensor(REAR_RIGHT) - rear_dist) / num_recordings;
		}
		Serial.print(front_dist); Serial.print("\t"); Serial.println(rear_dist);
		return false; //Don't move until we have some values
	}
	else //Update values using an exponential average
	{
		if(dir == LEFT)
		{
			front_dist += 0.3 * (wall_sensors_->ReadSensor(FRONT_LEFT) - front_dist);
			rear_dist += 0.3 * (wall_sensors_->ReadSensor(REAR_LEFT) - rear_dist);
		}
		else
		{
			front_dist += 0.3 * (wall_sensors_->ReadSensor(FRONT_RIGHT) - front_dist);
			rear_dist += 0.3 * (wall_sensors_->ReadSensor(REAR_RIGHT) - rear_dist);
		}
	}

	//Check if front sensor has detected a wall (only once)
	if(!front_detected_ && front_dist < sensor_gap_min_dist_)
	{
		front_detected_ = true; //set flag for rear sensor to start detection
	}
	//Check if rear sensor has detected a wall
	if(!rear_detected_ && front_detected_ && rear_dist < sensor_gap_min_dist_)
	{
		rear_detected_ = true; //Set flag to signal we've passed the wall		
	}
	//Go forward for a little bit so we clear our ass from the wall.
	if(rear_detected_)
	{
		if(motors_->GoStraight(clearing_time_))
		{
			front_detected_ = false;  //reset flags for next time
			rear_detected_ = false;
			motors_->StopPID(); //Stop PID for this use
			front_dist = -1.0f;
			rear_dist = -1.0f;
			num_recordings = 0;
			return true; //Return true to signal both sensors detected a wall.
		}
		else
		{
			return false;
		}
	}

	//Go straight using the encoders
	motors_->GoStraight();
	return false;
}

//Turns the motors 90 deg
bool Brain::Rotate90(Direction dir)
{
	return  motors_->Rotate(dir);
}

//Rotate the robot in a given direction until its front and rear IR sensors read the same value.
bool Brain::SquareToWall(Direction dir)
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

	//Check difference between front and rear sensors
	float diff = front_dist - rear_dist + squaring_offset_;
	//Check if theyre close enough to be the same; if so stop
	if(abs(diff) < squaring_diff_threshold_)
	{
		motors_->StopMotors();
		return true;
	}
	//Otherwise rotate based on which way we're facing and the sign of difference
	if(dir == LEFT)
	{
		if(diff < 0)
		{
			motors_->TurnStationary(motors_->drive_power_/2, RIGHT);
		}
		else //diff > 0
		{
			motors_->TurnStationary(motors_->drive_power_/2, LEFT);
		}
	}
	else //dir == RIGHT
	{
		if(diff < 0)
		{
			motors_->TurnStationary(motors_->drive_power_/2, LEFT);
		}
		else //diff > 0
		{
			motors_->TurnStationary(motors_->drive_power_/2, RIGHT);
		}
	}
	return false;
}

//Uses A* search to find optimal sequence of actions to go from current location to desired location
//Once sequence is found, the actions are executed.
//Returns:
//	ACT_GOING: still executing sequence of actions
//	ACT_SUCCESS: finished executing sequence of actions
//	ACT_ERROR: search provided no solution to the problem
//	other: Flags to indicate something went wrong
ActionResult Brain::GoToLocation(byte end_x, byte end_y, int desired_direction /*= -1*/)
{
	static bool is_searching = true;
	static ActionList action_list;
	//Perform the search and convert it to a usable list
	if(is_searching)
	{
		byte_action_list byte_actions = SearchAlgorithm::AStarGoAToB(end_x, end_y, robot_state_, board_state_, desired_direction);
		for(byte_action action : byte_actions)
		{
			SearchAlgorithm::PrintByteActionString(action);
		}
		action_list = ByteActionListConverter(byte_actions);
		is_searching = false;
		//If the action list is empty after search has been completed, no good!
		//This means our search found no solution; return error
		if(action_list.IsEmpty())
		{
			is_searching = true; //reset search for next time
			return ACT_ERROR;
		}
	}

	//If the list is empty, we've completed our execution of the actions
	if(action_list.IsEmpty())
	{
		is_searching = true; //reset search for next time
		return ACT_SUCCESS;
	}

	//Get current action and execute it
	Action* curr_action = action_list.GetCurrentAction();
//	curr_action->Print();
	ActionResult result = curr_action->Run();

	//From the result of the action, return things to the calling function
	switch(result)
	{
	case ACT_GOING: //Continue on this action
		return ACT_GOING;
		break;
	case ACT_SUCCESS: //Good! Action completed, Move on to the next step
		action_list.MoveToNextAction();
		return ACT_GOING;
		break;
	default: //Error; return error flags and let calling function deal with it. Do not continue with these actions
		is_searching = true; //reset search for next time
		return result;
		break;
	}
}