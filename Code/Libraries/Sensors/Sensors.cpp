#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables:
 */
VisualSensor::VisualSensor(VisualSensorConfig sensor_config)
{
	//loop through all the block counts and set to 0
	for (int block = 0; block < 2; block++)
	{
		blockCounts_[block] = 0;
	}

	config = sensor_config;

	//Initialize pixy
	pixy_.init();
	signature_ = 1;

}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{
}

boolean VisualSensor::IsGoodBlock(Block target_block)
{
	if (target_block.signature == 1 || target_block.signature == 2)
	{
		return true;
	}
	else return false;
}

/**
 * Determines the score of a block. Bigger, lower blocks that are close to the center will get the highest score
 */
float VisualSensor::GetBlockScore(Block block, boolean print)
{
	//find how close the block is to the center. Negative because we want the smallest distance to the center to have the most score
	float center = config.block_score_consts[0] * (abs(config.center - (int)block.x));
	float bottomLine = config.block_score_consts[1] * ((int)block.y + (int)block.height / 2); //find the bottom line of the lowest block (bigger y is closer to ground)
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
Block VisualSensor::GetBlock()
{
	Block block = BAD_BLOCK;

	float maxScore = -999999.0f;
	//Get the number of blocks(detected objects) from the pixy
	int numBlocks = pixy_.getBlocks();
	numBlocks += pixy_.getBlocks(); //For some reason GetBlocks needs to be called twice 

	 //Loop through all the blocks to find the best block to go to
	for(int blockIndex = 0; blockIndex < numBlocks; blockIndex++)
	{
		Block currBlock = pixy_.blocks[blockIndex];

		if(currBlock.signature == 1 || currBlock.signature == 2) //Check if this block is one we care about
		{
			float size = (currBlock.height * currBlock.width); //ignore blocks that are insignificant

			if(size > config.min_block_size) //If the block's size is big enough determine its score
			{
				float currScore = GetBlockScore(currBlock, false);

				//see if this is a max score
				if(currScore >= maxScore)
				{
					block = currBlock;
					maxScore = currScore;
				}
			}
		}
	}
	if(maxScore > config.min_block_score)
	{
		IncrementBlocks(block);
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
void VisualSensor::IncrementBlocks(Block block)
{
	blockCounts_[block.signature - 1]++; //signatures are 1 indexed but our array is 0 indexed
}

/**
* Returns the count of the block signature that the GetBlock method saw most often,
* and resets the counts back to 0 if desired
*/
byte VisualSensor::GetBlockSignature(boolean resetCounts)
{
	int maxCount = 0;
	int sig = 1;
	//loop through all the block counts
	for (int block = 0; block < 2; block++)
	{
		if (blockCounts_[block] >= maxCount) //If this block has a maximum number of blocks
		{
			maxCount = blockCounts_[block]; //Set the max count
			sig = block + 1; //Set the signature (+1 because it is 1 indexed)
		}
		if (resetCounts) blockCounts_[block] = 0; //reset sig counts
	}
	signature_ = sig;
	return blockCounts_[sig - 1];
}

//Read value from front IR sensor and convert it to cm. The distance measurement is accurate
//for close range(4 - 25cm) but gets innaccurate out of that range. 
//Far away readings are very noisy.
float VisualSensor::ReadProximity()
{
	int sensorVal = analogRead(config.ir_port);
	if(sensorVal < 19)
		return 999.0f; //sensor detects very far distance, return large float so it doesnt return a negative number
	else
		return 2410.6f / (sensorVal - 18.414f);
}

/**********
 ** GYRO **
 **********/
Gyro::Gyro()
{
	Wire.begin();

	if (!gyro_.init())
	{
		Serial.println("Failed to autodetect gyro_ type!");
		while (1);
	}
	gyro_.enableDefault();

	previous_time = 0UL;
	angleZ_ = 0.0f;
	offset_angle = 0.0;

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
float Gyro::GetDegrees()
{
	float correctAngle = angleZ_ + offset_angle;
	if (correctAngle > 360) correctAngle -= 360;
	if (correctAngle < 0) correctAngle += 360;
	return correctAngle;
}

/**
* Updates the current angle read by the gyro_. Should be called every loop. Takes in the current time of the loop in millis().
*/
void Gyro::Update(unsigned long current_time)
{
	gyro_.read();

	unsigned long sampleTime = current_time - previous_time;
	previous_time = current_time;

	//find current rate of rotation
	float rateZ = ((float)gyro_.g.z - calibration.averageBiasZ);
	if (abs(rateZ) < 3 * calibration.sigmaZ)
	{
		rateZ = 0.0f;
	}

	//find angle
	angleZ_ += (rateZ * sampleTime / 1000.0f) * calibration.scaleFactorZ; //divide by 1000(convert to sec)

	// Keep our angle between 0-359 degrees
	if (angleZ_ < 0) angleZ_ += 360;
	else if (angleZ_ >= 360) angleZ_ -= 360;

	// Serial.print("Angle = "); Serial.print(angleZ_); Serial.print("\tRate = "); Serial.println(rateZ * calibration.scaleFactorZ);
}

WallSensors::WallSensors(WallSensorsConfig wall_sensors_config)
{
	config = wall_sensors_config;
}

//Destructor
WallSensors::~WallSensors()
{

}

//Read value from one of the IR sensors and convert it to cm. The distance measurement is accurate
//for close range(4 - 25cm) but gets innaccurate out of that range. 
//Far away readings are very noisy.
float WallSensors::ReadSensor(SensorPosition pos)
{
	float sensorVal = 0.0;
	switch(pos)
	{
		case FRONT_LEFT:
			sensorVal = analogRead(config.front_left_sensor_pin);
			break;
		case FRONT_RIGHT:
			sensorVal = analogRead(config.front_right_sensor_pin);
			break;
		case REAR_LEFT:
			sensorVal = analogRead(config.rear_left_sensor_pin);
			break;
		case REAR_RIGHT:
			sensorVal = analogRead(config.rear_right_sensor_pin);
			break;
	}
	if(sensorVal < 19)
		return 999.0f; //sensor detects very far distance, return large float so it doesnt return a negative number
	else
		return 2410.6f / (sensorVal - 18.414f);
}