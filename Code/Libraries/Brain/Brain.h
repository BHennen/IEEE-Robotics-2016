#ifndef Brain_H
#define Brain_H

#include <Arduino.h>
#include "Sensors.h"
#include "Motors.h"
#include <list>
#include <functional>

//Board positions
enum Position
{
	START,
	RED,
	YELLOW,
	CROSSROAD,
	CITY_R,
	CITY_L,
	MEXICO,
	USA,
	FRONTIER,
	GRASS_S,
	GRASS_N
};

//Flags for wall-follow stopping conditions
enum StopConditions
{
	GAP = 1 << 0,
	PIXY = 1 << 1,
	FRONT = 1 << 2,
	NONE = 0
};

//Enum of what state the GoAtoB is in.
enum GoAToBState
{
	GOING,
	ERROR,
	STOP_GAP,
	STOP_PIXY,
	STOP_FRONT,
	SUCCESS
};

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
};

/**
 * Class that has high level functions that combine the sensors and motors to run the robot around the track successfully.
 */
class Brain
{
public:
	// Variables //////////////////////////////////////////

	BrainConfig config;

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

	//Combine FollowWall and turn functions to go to a position on the board. Returns true when it is there.
	GoAToBState GoAtoB(Position start_pos, Position end_pos);

private:

	// Variables ///////////////////////

	VisualSensor *visual_sensor_;
	WallSensors *wall_sensors_;
	Motors *motors_;
	Gyro *gyro_;

	bool front_detected_; //Bool to determine if front IR sensor has detected a gap.
	bool reset_pid_; //Bool to reset PID when we change why we're using it.
	float last_heading_; //Last heading of our robot (degrees).
	int good_block_count_; //How many consecutive goodblocks the pixy has seen when following a wall.

	// Functions //////////////////////
	//Search method to find path from one position on the board to another.
	//Returns a list of actions to take. Used something like:
	//
	
	//
	//Create a type that is a vector of functions that have a shared return type and arguments, call it action_list.
	//Will be used to store functions that will be executed in order.
	//typedef std::vector<std::function<return_type (args)>> action_list;
	//
	//Example use:
	//
	//Create functions that have arguments bounded to them, ready to be called.
	//auto f1 = std::bind(function_name1, arg1, arg2, ...);
	//auto f2 = std::bind(function_name2, arg1, arg2, ...);
	//
	//Create list:
	//action_list my_list;
	//Add functions to list:
	//my_list.push_back(f1);
	//my_list.push_back(f2);
	//
	//Loop through list and execute the function. (Shouldnt use for loop in arduino, however)
	//for(auto& f : my_list)
	//{
	//	f();
	//}
	std::list AStarSearch(); 
};

#endif