#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <EEPROM.h>
#include <L3G.h>

/**
 * Class that contains sensors and functions to be used to visually locate and detect a block in front of the robot.
 */
class VisualSensor
{
public:
	/**
	 * Constructor. Intitialize the following variables:
	 */
	VisualSensor(const char IRPort, int center, float IRConstantThreshold, float* blockScoreConsts, float minimumBlockScore, float minimumBlockSize);
		
	//Destructor
	~VisualSensor();

	//Check if the target block is valid.
	boolean isGoodBlock(Block targetBlock);

	/**
	* Determines the score of a block. Bigger, lower blocks that are close to the center will get the highest score
	*/
	float getBlockScore(Block block, boolean print);

	/**
	 * Find the correct block to go to. Returns BAD_BLOCK if no good blocks to go to.
	 */
	Block getBlock(unsigned long currentTime);

	/**
	* Returns the count of the block signature that the getblock method saw most often,
	* and resets the counts back to 0 if desired
	*/
	int getBlockSignature(boolean resetCounts);
	
	/**
	* Read the value from the IRsensor port (0 - 1023)
	*/
	int readProximity();

	/* Pixy Variables */
	//Variable that is a "bad block", used when we find no good blocks
	const Block BAD_BLOCK = {-1, -1, -1, -1, -1, -1};
	int _center; //Where the robot aims for in PID control. Also affects score of blocks

	/* IR Variables */
	float _IRaverage;
	boolean _IRisConstant;

private:
	/* Pixy Variables */
	Pixy _pixy; //Variable for pixy camera
	int blockCounts[2]; //Record how many times we've seen each block signature
	float* _blockScoreConsts; //These values are the weights used to determine a blocks score
	float _minimumBlockScore;
	float _minimumBlockSize;
	byte _signature;
	
	/* IR Variables */
	int _proximity;
	char _IRPort;
	float _IRConstantThreshold;

	/**
	* Increments how many times we've seen the given block
	*/
	void incrementBlocks(Block block);
};

/**
 * Struct for the Calibration Data for the gyro.
 */
struct CalibrationData
{
	float averageBiasZ;
	float sigmaZ;
	float scaleFactorZ;
};

/**
 * The gryo allows you to get the current heading of the robot in degrees.
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
	float getDegrees();

	/**
	 * Updates the current angle read by the gyro. Should be called every loop. Takes in the current time of the loop in millis().
	 */
	void update(unsigned long currentTime);

	double _offSetAngle; //Angle how much the gyro is offset
private:
	L3G gyro;
	float angleZ;
	unsigned long previousTime;
	CalibrationData calibration;
};

#endif