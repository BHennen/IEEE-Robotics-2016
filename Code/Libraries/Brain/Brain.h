#ifndef Brain_H
#define Brain_H

#include <Arduino.h>
#include "Sensors.h"
#include "Motors.h"
#include <BrainEnums.h>
#include <States.h>
#include <iterator>
#include <bitset>
#include <vector>
#include <AStarSearch.h>
#include <ActionList.h>
//#include <ActionList.cpp>
//Components the brain will use.
struct BrainModules
{
	VisualSensor *visual_sensor;
	WallSensors *wall_sensors;
	Motors *motors;
	Gyro *gyro;
};

//Values used to configure the brain.
struct BrainConfig
{
	//Variables for wall following
	float sensor_gap_min_dist;
	float desired_dist_to_wall;
	float front_sensor_stop_dist;
	byte pixy_block_detection_threshold;

	//Robot State configuration
	Direction init_direction;
	byte init_x;
	byte init_y;

	//Board state config
	std::bitset<BOARD_STATE_SIZE> init_board_state[64];
	//int init_board_state[64];
};

/**
 * Class that has high level functions that combine the sensors and motors to run the robot around the track successfully.
 */
class Brain
{
public:
	// Variables //////////////////////////////////////////

	RobotState robot_state_;
	BoardState board_state_;
	
	// Functions //////////////////////////////////////////

	/**
	 * Constructor. 
	 */
	Brain(BrainModules brain_modules, BrainConfig brain_config);
		
	//Destructor
	~Brain();

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
	StopConditions FollowWall(Direction dir, StopConditions flags);

	//Use pixy and other sensor to go to victim. Return true when it has stopped in the right position.
	bool GoToVictim();

	//Go straight until a wall has been detected by both the front and rear sensors and then stops.
	//Return true when it has done so, otherwise return false.
	bool TravelPastWall(Direction dir);

	//Turns the motors 90 deg (using the same function in motors, but allowed to be called from brain class.)
	bool Rotate90(Direction dir);

	//Combine FollowWall and turn functions to go to a position on the board. Returns true when it is there.
	GoAToBState GoAtoB(Position start_pos, Position end_pos);

private:

	// Variables ///////////////////////

	VisualSensor *visual_sensor_;
	WallSensors *wall_sensors_;
	Motors *motors_;
	Gyro *gyro_;


	//config variables used for wall following
	const float sensor_gap_min_dist_;
	const float desired_dist_to_wall_;
	const float front_sensor_stop_dist_;
	const byte pixy_block_detection_threshold_;

	bool front_detected_; //Bool to determine if front IR sensor has detected a gap.
	bool reset_pid_; //Bool to reset PID when we change why we're using it.
	float last_heading_; //Last heading of our robot (degrees).
	int good_block_count_; //How many consecutive goodblocks the pixy has seen when following a wall.

	// Functions //////////////////////

};

#endif