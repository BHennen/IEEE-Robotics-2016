#ifndef Brain_H
#define Brain_H

#include <Arduino.h>
#include "Sensors.h"
#include "Motors.h"

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
	FRONT = 1 << 2
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
	 * Use sensors to follow a wall to the [direction] of the robot indefinitely. Return true when any of the stop conditions set
	 * by flags are met.
	 * stop conditions :
	 *		GAP -- Check if gap was found in the direction d. Return true when both sensors have detected the gap.
	 *		PIXY -- Check if pixy has detected ~10? good blocks. Return true when it has detected enough blocks.
	 *		FRONT -- Check front IR sensor return true when it it close to wall in front.
	 *
	 * To call this: FollowWall(LEFT, GAP | PIXY); <-- This follow left wall and stop when a gap is detected or pixy sees a victim
	 */
	bool FollowWall(Direction dir, StopConditions flags);

	//Combine FollowWall and turn functions to go to a position on the board. Returns true when it is there.
	bool GoAtoB(Position A, Position B);

	//Use pixy and other sensor to go to victim. Return true when it has stopped in the right position.
	bool GoToVictim();


private:

	// Variables ///////////////////////

	VisualSensor *visual_sensor_;
	WallSensors *wall_sensors_;
	Motors *motors_;
	Gyro *gyro_;

	bool gap_started_; //Bool to determine if front IR sensor has detected a gap.
	bool reset_pid_; //Bool to reset PID when we change why we're using it.
	float last_heading_;  //Last heading of our robot (degrees).

	// Functions //////////////////////
};

#endif