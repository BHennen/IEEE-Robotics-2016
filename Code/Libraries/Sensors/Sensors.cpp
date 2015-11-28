#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables
 * _pixy: Initialize the pixy camera.
 * _IRPort: Set the IRPort.
 * _stopVoltage: Set the stop voltage; The robot should stop whenever the
 *               input voltage from the IR sensor is greater than this voltage.
 */
VisualSensor::VisualSensor(const char IRPort, const int stopVoltage, const int closeVoltage, byte errorVoltage, int peakVoltage,
	int center, byte errorDeadzone, unsigned long pixyStallTime, float IRConstantThreshold,
	float* blockScoreConsts, float* PIDconsts, float* pixyRotatePIDconsts, float minimumBlockScore, float minimumBlockSize, float maximumBlockY,
	byte getBlockSigCount)
{
	//loop through all the block counts and set to 0
	for (int block = 0; block < 2; block++)
	{
		blockCounts[block] = 0;
	}

	//Initialize pixy
	_pixy.init();
	_center = center;
	_pixyStallTime = pixyStallTime;
	_PIDconsts = PIDconsts;
	_pixyRotatePIDconsts = pixyRotatePIDconsts;
	_signature = 1;
	_getBlockSigCount = getBlockSigCount;

	//constants for determining a block's score
	_blockScoreConsts = blockScoreConsts;

	_minimumBlockScore = minimumBlockScore; //Used to determine how close to the center an acceptable block should be
	_minimumBlockSize = minimumBlockSize; //Used to determine how close to the center a centered block should be
	_maximumBlockY = maximumBlockY;

	//Set IR Port
	_IRPort = IRPort;
	_stopVoltage = stopVoltage;
	_closeVoltage = closeVoltage;
	_errorVoltage = errorVoltage;
	_peakVoltage = peakVoltage;
	_IRConstantThreshold = IRConstantThreshold;
	_isClose = false;
	_IRisConstant = false;
	_IRaverage = -1;

	_errorDeadzone = errorDeadzone;
}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{
}

/**
* Make sure everything is good to go before we start
*/
boolean VisualSensor::setup(unsigned long currentTime)
{
	//make sure the pixy can see some blocks before we go.
	static int numBlocksRead = 0;
	if (numBlocksRead < _getBlockSigCount)
	{
		if (isGoodBlock(getBlock(currentTime))) //read a block from pixy
		{
			numBlocksRead++;
		}
		return false;
	}
	else //once we've read enough blocks, we're done setting up
	{
		return true;
	}
}

boolean VisualSensor::isGoodBlock(Block targetBlock)
{
	if (targetBlock.signature == 1 || targetBlock.signature == 2)
	{
		return true;
	}
	else return false;
}

/**
 * Determines the score of a block. Bigger, lower blocks that are close to the center will get the highest score
 */
float VisualSensor::getBlockScore(Block block, boolean print)
{
	//find how close the block is to the center. Negative because we want the smallest distance to the center to have the most score
	float center = _blockScoreConsts[0] * (abs(_center - (int)block.x));
	float bottomLine = _blockScoreConsts[1] * ((int)block.y + (int)block.height / 2); //find the bottom line of the lowest block (bigger y is closer to ground)
	float score = (bottomLine - center); //factor in how low the block is, how close it is to center, and how far away we are

	if (print)
	{
		Serial.print("Sig = ");
		Serial.print(block.signature);
		Serial.print("\tY = ");
		Serial.print(block.y);
		Serial.print("\tCenter = ");
		Serial.print(center);
		Serial.print("\tBot Line = ");
		Serial.print(bottomLine);
		Serial.print("\tTotal = ");
		Serial.println(score);
	}
	return score;
}

/**
 * Find the correct block to go to. Returns BAD_BLOCK if no good block found.
 */
Block VisualSensor::getBlock(unsigned long currentTime)
{
	Block block = BAD_BLOCK;

	float maxScore = -999999999;
	//Get the number of blocks(detected objects) from the pixy
	int numBlocks = _pixy.getBlocks();
	numBlocks += _pixy.getBlocks(); //For some reason getBlocks needs to be called twice 

	 //Loop through all the blocks to find the best block to go to
	for(int blockIndex = 0; blockIndex < numBlocks; blockIndex++)
	{
		Block currBlock = _pixy.blocks[blockIndex];

		if(currBlock.signature == 1 || currBlock.signature == 2) //Check if this block is one we care about
		{
			float size = (currBlock.height * currBlock.width); //ignore blocks that are insignificant

			if(size > _minimumBlockSize) //If the block's size is big enough determine its score
			{
				float currScore = getBlockScore(currBlock, false);

				//see if this is a max score
				if(currScore >= maxScore)
				{
					block = currBlock;
					maxScore = currScore;
				}
			}
		}
	}
	if(maxScore > _minimumBlockScore)
	{
		incrementBlocks(block);
	}
	else
	{
		//This block didn't fit our criteria for a good enough block; return bad_block
		block = BAD_BLOCK;
	}
	return block;
}

/**
 * Increments how many times we've seen the given block
 */
void VisualSensor::incrementBlocks(Block block)
{
	blockCounts[block.signature - 1]++; //signatures are 1 indexed but our array is 0 indexed
}

/**
* Returns the count of the block signature that the getblock method saw most often,
* and resets the counts back to 0 if desired
*/
int VisualSensor::getBlockSignature(boolean resetCounts)
{
	int maxCount = 0;
	int sig = 1;
	//loop through all the block counts
	for (int block = 0; block < 2; block++)
	{
		if (blockCounts[block] >= maxCount) //If this block has a maximum number of blocks
		{
			maxCount = blockCounts[block]; //Set the max count
			sig = block + 1; //Set the signature (+1 because it is 1 indexed)
		}
		if (resetCounts) blockCounts[block] = 0; //reset sig counts
	}
	_signature = sig;
	return blockCounts[sig - 1];
}

/**
 * Reads the input from the IRSensor port (0-1023) every CHECK_MSEC.
 * Calculates and updates the exponential average, and determines if the IR signal is constant.
 */
void VisualSensor::updateIR(unsigned long currentTime)
{
	static const unsigned long CHECK_MSEC = 1UL;
	static unsigned long previousTime = currentTime;
	
	static const int numProximities = 10;
	static int numProxRead = 0;
	static int lastProximities[numProximities];
	static int lastSlopes[numProximities - 1];

	unsigned long deltaTime = currentTime - previousTime;
	
	if(deltaTime >= CHECK_MSEC)
	{
		_proximity = analogRead(_IRPort);
		numProxRead = (numProxRead == numProximities) ? numProxRead : numProxRead++;
		//find the exponential average
		const float newValueWeight = 0.5;
		_IRaverage = (_IRaverage > 0) ? (1 - newValueWeight) * _IRaverage + newValueWeight * _proximity : _proximity;

		//find if the IR is constant
		//if we have max amount of proximities stored
		static float slopeSum = 0.0f;
		if(numProxRead == numProximities)
		{
			slopeSum -= lastSlopes[0]; //Remove the oldest slope from the sum
			//Left shift values, removing the oldest one
			for(byte i = 1; i < numProximities; i++)
			{
				lastProximities[i - 1] = lastProximities[i]; //Left shift all old proximities, removing oldest one
				if(i < numProximities - 1)
				{
					lastSlopes[i - 1] = lastSlopes[i] //Left shift all old slopes
				}
			}
		}
		//add new proximity and slope to end
		lastProximities[numProxRead - 1] = _proximity; 
		if(numProxRead > 1) //at least 1 slope
		{
			//Calculate new slope
			lastSlopes[numProxRead - 2] = (_proximity - lastProximities[numProxRead - 2]) / deltaTime;
			//Add most recent slope to slope sum
			slopeSum += lastSlopes[numProxRead - 2];
			//Determine if the slope for this window is flat enough to be considered constant
			float slope = slopeSum / (numProxRead - 1.0f);
			_IRisConstant = (abs(slope) < _IRConstantThreshold);
		}
		else
		{
			_IRisConstant = false;
		}
		previousTime = currentTime;
	}
}

/**
 * Read the value from the IRsensor port (0 - 1023)
 */
int VisualSensor::readProximity()
{
	return _proximity;
}

/**********
 ** GYRO **
 **********/
Gyro::Gyro(float* PIDconsts, float* rotatePIDconsts)
{
	_PIDconsts = PIDconsts;
	_rotatePIDconsts = rotatePIDconsts;

	Wire.begin();

	if (!gyro.init())
	{
		Serial.println("Failed to autodetect gyro type!");
		while (1);
	}
	gyro.enableDefault();

	previousTime = 0UL;
	angleZ = 0.0f;
	_offSetAngle = 0.0;
	averageTimedBias = 0.0;

	//Read values from eeprom
	//Read averageBiasZ
	int nextAddress = sizeof(EEPROM.get(0, averageBiasZ));
	//read sigmaZ
	nextAddress += sizeof(EEPROM.get(nextAddress, sigmaZ));
	//read scaleFactor
	nextAddress += sizeof(EEPROM.get(nextAddress, scaleFactorZ));

	Serial.print("Bias: ");
	Serial.print(averageBiasZ);
	Serial.print("\tSigma: ");
	Serial.print(sigmaZ);
	Serial.print("\tScale Factor: ");
	Serial.println(scaleFactorZ);
}

//Destructor
Gyro::~Gyro()
{

}

/**
* Make sure everything is good to go before we start
*/
boolean Gyro::setup(unsigned long currentTime)
{
	angleZ = 0; //reset angle until we're ready
	return true; //gyro needs no setup
}

/**
* Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
*/
float Gyro::getDegrees()
{
	float correctAngle = angleZ + _offSetAngle;
	if (correctAngle > 360) correctAngle -= 360;
	if (correctAngle < 0) correctAngle += 360;
	return correctAngle;
}

/**
* Updates the current angle read by the gyro. Should be called every loop. Takes in the current time of the loop in millis().
*/
void Gyro::update(unsigned long currentTime)
{
	gyro.read();

	unsigned long sampleTime = currentTime - previousTime;
	previousTime = currentTime;

	//find current rate of rotation
	float rateZ = ((float)gyro.g.z - averageBiasZ); //-averagetimebias
	if (abs(rateZ) < 3 * sigmaZ)
	{
		rateZ = 0.0f;
	}

	//find angle
	angleZ += (rateZ * sampleTime / 1000.0f) * scaleFactorZ; //divide by 1000(convert to sec)

	// Keep our angle between 0-359 degrees
	if (angleZ < 0) angleZ += 360;
	else if (angleZ >= 360) angleZ -= 360;

	// Serial.print("Angle = "); Serial.print(angleZ); Serial.print("\tRate = "); Serial.println(rateZ * scaleFactorZ);
}