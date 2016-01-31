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

	ir_port_ = sensor_config.ir_port;
	center_ = sensor_config.center; //Where the robot aims for in PID control. Also affects score of blocks
	center_const_ = sensor_config.block_score_consts[0];
	bottom_line_const_ = sensor_config.block_score_consts[1];
	min_block_score_ = sensor_config.min_block_score;
	min_block_size_ = sensor_config.min_block_size;

	//Initialize pixy
	pixy_.init();
	signature_ = 1;

	min_good_bad_ratio_ = sensor_config.min_good_bad_ratio;
	victim_scan_time_ = sensor_config.victim_scan_time;

	victim_sensor_pin_ = sensor_config.victim_sensor_pin;
	pinMode(victim_sensor_pin_, INPUT);
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
	float center = center_const_ * (abs(center_ - (int)block.x));
	float bottomLine = bottom_line_const_ * ((int)block.y + (int)block.height / 2); //find the bottom line of the lowest block (bigger y is closer to ground)
	float score = (bottomLine - center); //factor in how low the block is, how close it is to center, and how far away we are

	if (print)
	{
		Serial.print(F("Sig = "));
		Serial.print(block.signature);
		Serial.print(F("\tY = "));
		Serial.print(block.y);
		Serial.print(F("\tCenter = "));
		Serial.print(center);
		Serial.print(F("\tBot Line = "));
		Serial.print(bottomLine);
		Serial.print(F("\tTotal = "));
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

			if(size > min_block_size_) //If the block's size is big enough determine its score
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
	if(maxScore > min_block_score_)
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
	int sensorVal = analogRead(ir_port_);
	if(sensorVal < 19)
		return 999.0f; //sensor detects very far distance, return large float so it doesnt return a negative number
	else
		return 2410.6f / (sensorVal - 18.414f);
}


//Scans (using the Pixy) for a victim in front of the robot and returns a number depending on situation:
//0: Scan completed and no victim
//1: Scan completed and victim
//2: Scan uncompleted
byte VisualSensor::ScanForVictim()
{
	unsigned long curr_time = micros();
	//Set timer
	if(timer_ == 0)
	{
		timer_ = curr_time;
	}
	//Done scanning
	if(curr_time - timer_ > victim_scan_time_)
	{
		timer_ = 0; //reset timer
		unsigned int good_bad_ratio = num_good_scanned_ / num_bad_scanned_;
		num_good_scanned_ = 0;
		num_bad_scanned_ = 0;
		if(good_bad_ratio >= min_good_bad_ratio_)
		{
			return static_cast<byte>(1); //victim found
		}
		else
		{
			return static_cast<byte>(0); //victim not found
		}
	}
	//Scan next block
	if(IsGoodBlock(GetBlock()))
	{
		num_good_scanned_++;
	}
	else
	{
		num_bad_scanned_++;
	}
	return static_cast<byte>(2); //keep scanning
}

//Return whether or not there is a victim in the cutout of the robot
//TODO: Might have to account for a noisy signal.
bool VisualSensor::HasVictim()
{
	return digitalRead(victim_sensor_pin_);
}

//return center value 
int VisualSensor::GetCenter()
{
	return center_;
}

/**********
 ** GYRO **
 **********/
Gyro::Gyro()
{
	Wire.begin();

	if (!l3g_gyro_.init())
	{
		Serial.println(F("Failed to autodetect gyro type!"));
		while (1);
	}
	l3g_gyro_.enableDefault();

	previous_time = 0UL;
	angleZ_ = 0.0f;
	offset_angle = 0.0;

	//Read calibration values from eeprom
	EEPROM.get(0, calibration);

	Serial.print(F("Bias: "));
	Serial.print(calibration.averageBiasZ);
	Serial.print(F("\tSigma: "));
	Serial.print(calibration.sigmaZ);
	Serial.print(F("\tScale Factor: "));
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
void Gyro::Update()
{
	l3g_gyro_.read();

	unsigned long current_time = micros();
	unsigned long sampleTime = current_time - previous_time;
	previous_time = current_time;

	//find current rate of rotation
	float rateZ = ((float)l3g_gyro_.g.z - calibration.averageBiasZ);
	if (abs(rateZ) < 3 * calibration.sigmaZ)
	{
		rateZ = 0.0f;
	}

	//find angle
	angleZ_ += (rateZ * sampleTime / 1000000.0f) * calibration.scaleFactorZ; //divide by 1000000.0(convert to sec)

	// Keep our angle between 0-359 degrees
	if (angleZ_ < 0) angleZ_ += 360;
	else if (angleZ_ >= 360) angleZ_ -= 360;

	// Serial.print(F("Angle = ")); Serial.print(angleZ_); Serial.print(F("\tRate = ")); Serial.println(rateZ * calibration.scaleFactorZ);
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