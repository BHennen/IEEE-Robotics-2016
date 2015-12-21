#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables:
 */
VisualSensor::VisualSensor(const char IRPort, int center, float IRConstantThreshold, float* blockScoreConsts, float minimumBlockScore, float minimumBlockSize)
{
	//loop through all the block counts and set to 0
	for (int block = 0; block < 2; block++)
	{
		blockCounts[block] = 0;
	}

	//Initialize pixy
	_pixy.init();
	_center = center;
	_signature = 1;

	//constants for determining a block's score
	_blockScoreConsts = blockScoreConsts;

	_minimumBlockScore = minimumBlockScore; //Used to determine how close to the center an acceptable block should be
	_minimumBlockSize = minimumBlockSize; //Used to determine how close to the center a centered block should be

	//Set IR Port
	_IRPort = IRPort;
	_IRConstantThreshold = IRConstantThreshold;
	_IRisConstant = false;
	_IRaverage = -1;
}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{
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
 * Read the value from the IRsensor port (0 - 1023)
 */
int VisualSensor::readProximity()
{
	return analogRead(_IRPort);
}

/**********
 ** GYRO **
 **********/
Gyro::Gyro()
{
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

	//Read calibration values from eeprom
	EEPROM.get(0, calibration);

	Serial.print("Bias: ");
	Serial.print(calibration.averageBiasZ);
	Serial.print("\tSigma: ");
	Serial.print(calibration.sigmaZ);
	Serial.print("\tScale Factor: ");
	Serial.println(calibration.scaleFactorZ);
}

//Destructor
Gyro::~Gyro()
{

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
	float rateZ = ((float)gyro.g.z - calibration.averageBiasZ);
	if (abs(rateZ) < 3 * calibration.sigmaZ)
	{
		rateZ = 0.0f;
	}

	//find angle
	angleZ += (rateZ * sampleTime / 1000.0f) * calibration.scaleFactorZ; //divide by 1000(convert to sec)

	// Keep our angle between 0-359 degrees
	if (angleZ < 0) angleZ += 360;
	else if (angleZ >= 360) angleZ -= 360;

	// Serial.print("Angle = "); Serial.print(angleZ); Serial.print("\tRate = "); Serial.println(rateZ * calibration.scaleFactorZ);
}