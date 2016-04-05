#ifndef Brain_H
#define Brain_H

#include <Arduino.h>
#include "Sensors.h"
#include "Motors.h"
#include <BrainEnums.h>
#include <States.h>
#include <iterator>
#include <Bitset.h>
#include <vector>
#include <AStarSearch.h>
#include <BrainActions.h>
#include <ActionList.h>

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
	float min_dist_to_wall;
	float front_sensor_stop_dist;
	byte pixy_block_detection_threshold;
	float squaring_diff_threshold;
	unsigned long clearing_time; //How long to go past a gap or wall so we clear the rear end.
	float squaring_offset;


	//Robot State configuration
	Direction init_direction;
	byte init_x;
	byte init_y;

	//Board state config
	byte init_board_state[8][8];
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
	bool has_victim;
	byte num_victims;
	bool done_moving;
	byte victim_sig;
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

	//Rotate the robot in a given direction until its front and rear IR sensors read the same value.
	bool SquareToWall(Direction dir);

	//Runs code to drop off a victim.
	bool DropOffVictim();

	//Uses A* search to find optimal sequence of actions to go from current location to desired location
	//Once sequence is found, the actions are executed.
	ActionResult GoToLocation(byte end_x, byte end_y, int desired_direction = -1);

private:
	bool sweep = false;
	// Variables ///////////////////////
	VisualSensor *visual_sensor_;
	WallSensors *wall_sensors_;
	Motors *motors_;
	Gyro *gyro_;

	//config variables used for wall following
	volatile float front_dist = -1.0;
	volatile float rear_dist = -1.0;
	const float sensor_gap_min_dist_;
	const float desired_dist_to_wall_;
	float min_dist_to_wall_;
	const float front_sensor_stop_dist_;
	const byte pixy_block_detection_threshold_;
	const float squaring_diff_threshold_;
	float squaring_offset_;

	unsigned long clearing_time_;
	bool front_detected_; //Bool to determine if front IR sensor has detected a gap.
	bool rear_detected_ = false;
	float last_heading_; //Last heading of our robot (degrees).
	int good_block_count_; //How many consecutive goodblocks the pixy has seen when following a wall.

	// Functions //////////////////////
	//Convert a list byte_action's to a list of Actions (which take more memory but are easier to handle
	ActionList ByteActionListConverter(byte_action_list a_star_results);
};

#endif