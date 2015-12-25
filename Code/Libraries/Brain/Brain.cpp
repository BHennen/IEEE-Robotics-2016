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

	gap_started_ = false;
	reset_pid_ = true;
	last_heading_ = 0.0;
}

//Destructor
Brain::~Brain()
{

}

/**
* Use sensors to follow a wall to the [direction] of the robot indefinitely. Return true when any of the stop conditions set
* by flags are met.
* stop conditions :
*		GAP -- Check if gap was found in the direction d. Return true when both sensors have detected the gap.
*		PIXY -- Check if pixy has detected ~10? good blocks. Return true when it has detected enough blocks.
*		FRONT -- Check front IR sensor return true when it it close to wall in front.
*
* To call this: FollowWall(LEFT, GAP | PIXY); <-- This will follow left wall and stop when a gap is detected or pixy sees a victim
*/
bool Brain::FollowWall(Direction dir, StopConditions flags)
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
		this->gap_started_ = false;  //reset gap_started_ flag for next time
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
			return true;
		}
	}

	//check for gap 
	if(flags & GAP)
	{
		//Check if front sensor has detected a gap (only once)
		if(!gap_started_ && front_dist > config.sensor_gap_min_dist)
		{
			gap_started_ = true; //set flag for rear sensor to start detection
			reset_pid_ = true; //Motors go straight using gyro after gap detected; reset PID
			last_heading_ = gyro_->GetDegrees(); //Save most recent heading so we can continue to go straight using gyro
		}
		//Check if rear sensor has detected a gap (only once)
		if(gap_started_ && rear_dist > config.sensor_gap_min_dist)
		{
			Reset();
			return true; //Return true to signal we arrived at stop condition
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
				return true;
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
	if(gap_started_)
	{
		motors_->FollowHeading(last_heading_);
		return false;
	}

//TODO: Update PID values
	//Follow Wall
	//For now, ignore rear sensor reading and try to maintain the desired distance from the wall using front sensor
	if(dir == RIGHT)
		motors_->GoUsingPIDControl(config.desired_dist_to_wall, front_dist, 1, 0, 0);
	else
		motors_->GoUsingPIDControl(front_dist, config.desired_dist_to_wall, 1, 0, 0); //Invert desired & current for left wall

	return false;
}

//Combine FollowWall and turn functions to go to a position on the board. Returns true when it is there.
bool Brain::GoAtoB(Position A, Position B)
{
	return false;
}

//Use pixy and other sensor to go to victim. Return true when it has stopped in the right position.
bool Brain::GoToVictim()
{
	return false;
}
