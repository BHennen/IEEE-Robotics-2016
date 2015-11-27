#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables
 * badBlock: Initialize all values of the block to -1.
 * _pixy: Initialize the pixy camera.
 * _IRPort: Set the IRPort.
 * _stopVoltage: Set the stop voltage; The robot should stop whenever the
 *               input voltage from the IR sensor is greater than this voltage.
 */
VisualSensor::VisualSensor(const char IRPort, const int stopVoltage, const int closeVoltage, byte errorVoltage, int peakVoltage,
	int center, byte errorDeadzone, unsigned long pixyStallTime, float IRConstantThreshold,
	float* blockScoreConsts, float* PIDconsts, float* pixyRotatePIDconsts, float minimumBlockScore, float minimumBlockSize, float maximumBlockY,
	byte getFishSigCount)
{
	//Set all the badBlock's values to -1
	badBlock.signature = 69;
	badBlock.x = -1;
	badBlock.y = -1;
	badBlock.width = -1;
	badBlock.height = -1;

	//Set the initial closest block value
	closestBlock.y = 199;
	//loop through all the block counts and set to 0
	for (int block = 0; block < 4; block++)
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
	_getFishSigCount = getFishSigCount;

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
	if (numBlocksRead < _getFishSigCount)
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
	if (targetBlock.signature == 1 || targetBlock.signature == 2 || targetBlock.signature == 3 || targetBlock.signature == 4)
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
	float bottomLine = _blockScoreConsts[1] * ((int)block.y + (int)block.height / 2); //find the bottom line of the lowest fish (bigger y is closer to ground)
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
 * Find the correct block to go to.
 * 1) Loop through all the blocks the pixy currently sees and find its score.
 * 2) If the maximum score is greater than the previous maximum score of all the previous function calls, go
 *    to that block.
 * 3) If reset is true, reset the max score to go to a different block.
 * 4) Increments the number of times the block with the highest score is seen.
 */
Block VisualSensor::getBlock(unsigned long currentTime)
{
	Block block = badBlock;

	float maxScore = -999999999;
	//Get the number of blocks(detected objects) from the pixy
	int numBlocks = _pixy.getBlocks();
	numBlocks += _pixy.getBlocks(); //For some reason getBlocks needs to be called twice 

	if (numBlocks == 0)
	{
		return block;
	}

	//Loop through all the blocks
	for (int blockIndex = 0; blockIndex < numBlocks; blockIndex++)
	{
		//find current block score
		Block currBlock = _pixy.blocks[blockIndex];
		if (currBlock.signature < 1 || currBlock.signature > 4)
		{
			currBlock.signature = badBlock.signature;
		}

		if (currBlock.signature != badBlock.signature)
		{
			float size = (currBlock.height * currBlock.width); //find the size of the fish (ignore blocks that are insignificant)

			if (size > _minimumBlockSize) //If the block's size is big enough determine its score
			{
				float currScore = getBlockScore(currBlock, false);

				//see if this is a max score
				if (currScore >= maxScore)
				{
					block = currBlock;
					maxScore = currScore;
				}
			}
		}
	}
	if (block.signature != badBlock.signature)
	{
		if (maxScore > _minimumBlockScore)
		{
			//getBlockScore(block, true);
			incrementBlocks(block);
			return block;
		}
		else
		{
			return badBlock;
		}
	}
	return badBlock;
}

/**
 * Increments how many times we've seen the given block
 */
void VisualSensor::incrementBlocks(Block block)
{
	if (block.signature != badBlock.signature) blockCounts[block.signature - 1]++; //signatures are 1 indexed but our array is 0 indexed
}

/**
* Returns the count of the fish signature that the getblock method saw most often,
* and resets the counts back to 0 if desired
*/
int VisualSensor::getFishSignature(boolean resetCounts)
{
	int maxCount = 0;
	int sig = 1;
	//loop through all the block counts
	for (int block = 0; block < 4; block++)
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
 * Reads the input from the IRSensor port. This number is from 0-1023,
 * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is
 * in the voltage range we want.
 */
void VisualSensor::update(unsigned long currentTime)
{
	static unsigned long previousTime = currentTime;
	static boolean debouncedIR = false;
	const unsigned long CHECK_MSEC = 1;
	//const unsigned long PRESS_MSEC = 5;
	//const unsigned long RELEASE_MSEC = 3;

	const int numProximities = 10;
	static int lastProximities[numProximities];
	static int lastSlopes[numProximities - 1];
	if (currentTime - previousTime >= CHECK_MSEC)
	{
		_proximity = analogRead(_IRPort);

		//find the exponential average
	
		const float newValueWeight = 0.5;
		_IRaverage = (_IRaverage > 0) ? (1 - newValueWeight) * _IRaverage + newValueWeight * _proximity : _proximity;

		//find if the IR is constant
		float temp = lastProximities[0];
		lastProximities[0] = _proximity;
		//Shift in the newest proximity
		for (byte i = 1; i < numProximities; i++)
		{
			float temp2 = lastProximities[i];
			lastProximities[i] = temp;
			temp = temp2;
		}
		int slopeSum = 0;
		for (byte i = 0; i < numProximities-1; i++)
		{
			lastSlopes[i] = lastProximities[i] - lastProximities[i + 1];
			slopeSum += lastSlopes[i];
		}
		float slope = slopeSum / (numProximities - 1.0f);
		_IRisConstant = (abs(slope) < _IRConstantThreshold);

		////find it the IR is pressed
		//previousTime = currentTime;
		//static uint8_t count = RELEASE_MSEC / CHECK_MSEC;
		//
		//int voltage = readProximity();
		//int error = _stopVoltage - voltage;
		//boolean rawState = (abs(error) < _errorVoltage);
		//if (rawState == debouncedIR)
		//{
		//	// Set the timer which allows a change from current state.
		//	if (debouncedIR) count = RELEASE_MSEC / CHECK_MSEC;
		//	else count = PRESS_MSEC / CHECK_MSEC;
		//}
		//else
		//{
		//	// Key has changed - wait for new state to become stable.
		//	if (--count == 0) {
		//		// Timer expired - accept the change.
		//		debouncedIR = rawState;
		//		_isClose = debouncedIR;
		//		// And reset the timer.
		//		if (debouncedIR) count = RELEASE_MSEC / CHECK_MSEC;
		//		else count = PRESS_MSEC / CHECK_MSEC;
		//	}
		//}
	}
	//_isClose = debouncedIR;
}

boolean VisualSensor::detectIRChange(unsigned long currentTime)
{
	static unsigned long previousTime = 0;
	int dt = currentTime - previousTime;
	previousTime = currentTime;
	static boolean reset = true;
	static int numSamples = 0;
	static float averageVal = 0.0f;
	static float M2 = 0.0f;
	static float avgDeltaAvg = 0.0f;
	static float M3 = 0.0f;
	const float breakPoint = 4.0f;
	const float newValWeight = 0.05f;
	const int sampleAmount = 5;
	const float threshold = 500;

	//measure a new sample
	int currReading = analogRead(_IRPort);

	//after a change point, reset the values
	if (reset)
	{
		Serial.println("reset");
		numSamples = 0;
		averageVal = currReading;
		avgDeltaAvg = 0.0;
		M2 = 0.0;
		M3 = 0.0;
		reset = false;
	}

	numSamples++;
	float prevAvg = averageVal;

	//calculate new average value
	float delta = currReading - averageVal;
	averageVal = (1 - newValWeight) * averageVal + newValWeight * currReading; //calculate exponential weighted average
	float newDelta = currReading - averageVal;

	//calculate avg rate of change of the average
	float dtAvg = (dt != 0) ? (averageVal - prevAvg) / dt : 0;
	float deltaAvg = dtAvg - avgDeltaAvg;
	avgDeltaAvg = (1 - newValWeight) * avgDeltaAvg + newValWeight * dtAvg;
	float newDeltaAvg = dtAvg - avgDeltaAvg;

	M2 += delta * newDelta;
	M3 += deltaAvg * newDeltaAvg;

	//calculate sigma, the standard deviation of the avg values
	float variance = M2 / (numSamples - 1);
	float sigma = sqrt(variance);
	float avgVariance = M3 / (numSamples - 1);
	float avgSigma = sqrt(avgVariance);

	Serial.print(currReading);
	Serial.print("\t");
	Serial.print(averageVal);
	Serial.print("\t");
	Serial.print(sigma);
	Serial.print("\t\t");
	Serial.print(dtAvg);
	Serial.print("\t");
	Serial.print(avgDeltaAvg);
	Serial.print("\t");
	Serial.println(avgSigma);

	if (abs(newDeltaAvg / avgSigma) > breakPoint)
	{
		reset = true;
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Read the value from the IRsensor port and converts it into a value from 0.0 - 5.0 volts.
 */
int VisualSensor::readProximity()
{
	return _proximity;
}

//Constructor
Gyro::Gyro(float* PIDconsts, float* rotatePIDconsts)
{
	_PIDconsts = PIDconsts;
	_rotatePIDconsts = rotatePIDconsts;

	Serial.println("beginning gyro...");
	Wire.begin();
	Serial.println("Done!");

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