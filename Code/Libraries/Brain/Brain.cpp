#include "Brain.h"

//Convert byte_action_list to vector::<Action*> and return it
ActionList Brain::ByteActionListConverter(byte_action_list a_star_results)
{
	ActionList action_list;
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
		if(prog == 0)
		{
			//rotate
			action_list.AddAction(new Rotate90Action(this, dir));
		}
		else if(prog == 1)
		{
			action_list.AddAction(new TravelPastWallAction(this, dir));
			//tpw
		}
		else if(prog == 2)
		{
			//follow wall			
			word err = action.Test(11) * 4 + action.Test(10) * 2 + action.Test(9) * 1;
			word suc = action.Test(8) * 4 + action.Test(7) * 2 + action.Test(6) * 1;
			
			StopConditions err_flags = static_cast<StopConditions>(err);
			StopConditions suc_flags = static_cast<StopConditions>(suc);

			action_list.AddAction(new FollowWallAction(this, dir, suc_flags, err_flags));
		}
		else if(prog == 3)
		{
			//gotovictim
			action_list.AddAction(new GoToVictimAction(this));
		}
	}
	return action_list;
}

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

//Turns the motors 90 deg, and updates the state when it has done so.
bool Brain::Rotate90(Direction dir)
{
	return motors_->Turn90(dir);
}

//Uses A* search to find optimal sequence of actions to go from current location to desired location
//Once sequence is found, the actions are executed.
//Returns:
//	ACT_GOING: still executing sequence of actions
//	ACT_SUCCESS: finished executing sequence of actions
//	ACT_ERROR: search provided no solution to the problem
//	other: Flags to indicate something went wrong
ActionResult Brain::GoToLocation(byte end_x, byte end_y)
{
	static bool is_searching = true;
	static ActionList action_list;
	//Perform the search and convert it to a usable list
	if(is_searching)
	{
		byte_action_list byte_actions = SearchAlgorithm::AStarGoAToB(end_x, end_y, robot_state_, board_state_);
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
	ActionResult result = curr_action->Run();

	//From the result of the action, return things to the calling function
	switch(result)
	{
	case ACT_GOING: //Continue on this action
		return ACT_GOING;
		break;
	case ACT_SUCCESS: //Good! Move on to the next step, and keep going
		action_list.MoveToNextAction();
		return ACT_GOING;
		break;
	default: //Error; return error flags and let calling function deal with it. Do not continue with these actions
		is_searching = true; //reset search for next time
		return result;
		break;
	}
}

