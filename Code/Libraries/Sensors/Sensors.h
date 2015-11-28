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
	 * Constructor. Intitialize the following variables
	 * _pixy: Initialize the pixy camera.
	 * _IRPort: Set the IRPort.
	 * _stopVoltage: Set the stop voltage; The robot should stop whenever the
	 *               input voltage from the IR sensor is greater than this voltage.
	 */
	VisualSensor(const char IRPort, const int stopVoltage, const int closeVoltage, byte errorVoltage, int peakVoltage,
		int center, byte errorDeadzone, unsigned long pixyUpdateTime, float IRConstantThreshold,
		float* blockScoreConsts, float* PIDconsts, float* pixyRotatePIDconsts, float minimumBlockScore, float minimumBlockSize, float maximumBlockY,
		byte getBlockSigCount);

	
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
	* Reads the input from the IRSensor port (0-1023) every CHECK_MSEC.
	* Calculates and updates the exponential average, and determines if the IR signal is constant.
	*/
	void updateIR(unsigned long currentTime);
	
	/**
	* Read the value from the IRsensor port (0 - 1023)
	*/
	int readProximity();

	//Variable that is a "bad block", used when we find no good blocks
	const Block BAD_BLOCK = {.signature = -1, .x = -1, .y = -1, .width = -1, .height = -1, .angle = -1};
	int _center; //Where the robot aims for in PID control. Also judges which block to go to
	float _minimumBlockScore;
	float _minimumBlockSize;
	float _maximumBlockY;
	float* _PIDconsts;
	float* _pixyRotatePIDconsts;
	byte _signature;
	int _stopVoltage; 
	int _closeVoltage; 
	float _IRaverage;
	boolean _IRisConstant;
	byte _errorDeadzone;
	unsigned long _pixyStallTime;
	boolean _isClose;
	byte _errorVoltage;
	int _peakVoltage;
	float _IRConstantThreshold;
private:
	int _proximity;
	float getHypotenuse(Block block); //Finds the hypotenuse of the block's x and y coordinates to the center bottom of the pixy
	Pixy _pixy; //Variable for pixy camera
	char _IRPort; //The port for the IR sensor
	int blockCounts[2]; //Record how many times we've seen each block signature
	Block closestBlock;
	//These values are the weights used to determine a blocks score
	float* _blockScoreConsts;

	byte _getBlockSigCount;



	/**
	* Increments how many times we've seen the given block
	*/
	void incrementBlocks(Block block);
};

/**
 * The gryo allows you to get the current heading of the robot in degrees.
 */
class Gyro
{
public:
	//Constructor
	Gyro(float* PIDconsts, float* rotatePIDconsts);

	//Destructor
	~Gyro();

	/**
	* Make sure everything is good to go before we start
	*/
	boolean setup(unsigned long currentTime);

	/**
	 * Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
	 */
	float getDegrees();

	/**
	 * Updates the current angle read by the gyro. Should be called every loop. Takes in the current time of the loop in millis().
	 */
	void update(unsigned long currentTime);

	float* _PIDconsts;
	float* _rotatePIDconsts;

	double _offSetAngle; //Angle how much the gyro is offset
	double averageTimedBias;
private:
	L3G gyro;
	float angleZ;
	unsigned long previousTime;

	float averageBiasZ;
	float sigmaZ;
	float scaleFactorZ;
};

#endif