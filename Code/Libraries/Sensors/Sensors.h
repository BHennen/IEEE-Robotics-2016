#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <L3G.h>

struct VisualSensorConfig
{
	byte ir_port;
	int center; //Where the robot aims for in PID control. Also affects score of blocks
	float* block_score_consts; //These values are the weights used to determine a blocks score
	float min_block_score;
	float min_block_size;
};

/**
 * Class that contains sensors and functions to be used to visually locate and detect a block in front of the robot.
 */
class VisualSensor
{
public:
	/**
	* Variables
	*/

	VisualSensorConfig config;
	//Variable that is a "bad block", used when we find no good blocks
	const Block BAD_BLOCK = {-1, -1, -1, -1, -1, -1};

	/**
	 * Functions
	 */

	//Constructor
	VisualSensor(VisualSensorConfig config);
		
	//Destructor
	~VisualSensor();

	//Check if the target block is valid.
	bool IsGoodBlock(Block target_block);

	/**
	* Determines the score of a block. Bigger, lower blocks that are close to the center will get the highest score
	*/
	float GetBlockScore(Block block, boolean print);

	/**
	 * Find the correct block to go to. Returns BAD_BLOCK if no good blocks to go to.
	 */
	Block GetBlock(unsigned long current_time);

	/**
	* Returns the count of the block signature that the GetBlock method saw most often,
	* and resets the counts back to 0 if desired
	*/
	byte GetBlockSignature(boolean resetCounts);
	
	/**
	* Read the value from the IRsensor port (0 - 1023)
	*/
	int ReadProximity();

private:
	/**
	* Variables
	*/

	/* Pixy Variables */
	Pixy pixy_; //Variable for pixy camera
	int blockCounts_[2]; //Record how many times we've seen each block signature
	byte signature_; //Most frequent signature last seen

	/**
	* Functions
	*/

	/**
	* Increments how many times we've seen the given block
	*/
	void IncrementBlocks(Block block);
};

/**
 * Struct for the Calibration Data for the gyro_.
 */
struct CalibrationData
{
	float averageBiasZ;
	float sigmaZ;
	float scaleFactorZ;
};

/**
 * The gyro_ allows you to get the current heading of the robot in degrees.
 */
class Gyro
{
public:
	//Constructor
	Gyro();

	//Destructor
	~Gyro();

	/**
	 * Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
	 */
	float GetDegrees();

	/**
	 * Updates the current angle read by the gyro_. Should be called every loop. Takes in the current time of the loop in millis().
	 */
	void Update(unsigned long current_time);

	float offset_angle; //Angle how much the gyro_ is offset
private:
	L3G gyro_;
	float angleZ_;
	unsigned long previous_time;
	CalibrationData calibration;
};

#endif