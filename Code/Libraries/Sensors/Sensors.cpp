#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables:
 */
VisualSensor::VisualSensor(VisualSensorConfig sensor_config)
{
	//loop through all the block counts and set to 0
	for(int block = 0; block < 2; block++)
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

	min_good_bad_ratio_ = sensor_config.min_good_bad_ratio;
	victim_scan_time_ = sensor_config.victim_scan_time;

	victim_sensor_pin_ = sensor_config.victim_sensor_pin;
	pinMode(victim_sensor_pin_, INPUT);
}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{}

boolean VisualSensor::IsGoodBlock(Block target_block)
{
	if(target_block.signature == 1 || target_block.signature == 2)
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

	if(print)
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
* Returns the block signature that the GetBlock method saw most often,
* and resets the counts back to 0 if desired
*/
byte VisualSensor::GetBlockSignature(boolean resetCounts)
{
	int maxCount = 0;
	int sig = 1;
	//loop through all the block counts
	for(int block = 0; block < 2; block++)
	{
		if(blockCounts_[block] >= maxCount) //If this block has a maximum number of blocks
		{
			maxCount = blockCounts_[block]; //Set the max count
			sig = block + 1; //Set the signature (+1 because it is 1 indexed)
		}
		if(resetCounts) blockCounts_[block] = 0; //reset sig counts
	}
	return sig;
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
Gyro::Gyro(byte threshold_size)
{
	Wire.begin();
	if(!l3g_gyro_.init())
	{
		Serial.println(F("Failed to autodetect gyro type!"));
		while(1);
	}
	l3g_gyro_.enableDefault(threshold_size);
	
	previous_time = 0UL;
	sample_time = 0UL;
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

	//Make sure the fifo is cleared (so an interrupt can be enabled)
	while((l3g_gyro_.readReg(L3G::FIFO_SRC) & B00011111) > 10)
	{
		l3g_gyro_.read();
	}
}

//Destructor
Gyro::~Gyro()
{

}

void Gyro::TransformData()
{
	//find current rate of rotation
	l3g_gyro_.read();//Read current data

	//Set time between samples.
	unsigned long current_time = micros();
	sample_time = current_time - previous_time; //time between getting new data
	previous_time = current_time;

	//get calibrated rate
	float rateZ = ((float)l3g_gyro_.z - calibration.averageBiasZ);

	//If we're 93.75% sure (according to Chebyshev) that this data is caused by normal fluctuations, ignore it.
	if(abs(rateZ) < 4 * calibration.sigmaZ)
	{
		rateZ = 0.0f;
	}

	//find angle and scale it to make sense
	angleZ_ += (rateZ * (sample_time / 1000000.0f)) * calibration.scaleFactorZ; //divide by 1000000.0(convert to sec)

	// Add offset and keep our angle between 0-359 degrees
	//angleZ_ + offset_angle; Dont use this for now
	while(angleZ_ < 0) angleZ_ += 360;
	while(angleZ_ >= 360) angleZ_ -= 360;
}

/**
* Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
*/
float Gyro::GetDegrees()
{
	if(l3g_gyro_.fresh_data) TransformData(); //if we've read raw data recently, calibrate it before returning angle
	return angleZ_;
}

/**
* Updates the current raw z read by the gyro_ and the time it took to read between current sample and last.
*/
void Gyro::Update()
{
	//Signal to update the data
	l3g_gyro_.fresh_data = true;
}

bool Gyro::Calibrate()
{
	//setup variables used in the calibration
	enum CalibrationStates
	{
		CalculateBias,
		MeasureScaleFactor,
		PrintDirections,
		CheckResults
	};
	const float ROTATION_ANGLE = 360.0f; //How far to rotate in calibration procedure
	const float READING_TO_DPS = 0.00875f; //Number to convert raw data to degrees
	static float STOP_RATE; //How slow to be considered stopped
	const unsigned long averagingTime = 5000000UL; //Optimal time based on allan variance is 5 sec

	static float scaleFactor = 0.0f;
	static float averageBiasZ = 0.0f;
	static float sigmaZ = 0.0f;
	static float angleZ = 0.0f;
	static float maxAngleZ = 0.0f;
	static unsigned int numSamples = 0;
	static float M2 = 0.0f;
	static unsigned long timer = 0;
	static bool printResults = true;
	static bool measureScaleFactor = true;
	static bool hasRotated = false;
	static bool hasPrinted = false;
	static CalibrationStates current_state = CalculateBias;

	unsigned long current_time = micros(); //read current time at every loop
	static unsigned long prev_time = current_time;
	unsigned long sampleTime = current_time - prev_time;
	bool data_avail = l3g_gyro_.read();
	prev_time = current_time;
	switch(current_state)
	{
	case CalculateBias:
		//Check if we have calculated bias for the desired amount of time.
		if(timer == 0)
		{
			Serial.println(F("Calculating Bias"));
			timer = current_time;
		}
		if(current_time - timer < averagingTime)
		{
			//Keep a running average of current value of the gyro
			if(data_avail)
			{
				numSamples++;
				float delta = (float)l3g_gyro_.z - averageBiasZ;
				averageBiasZ += delta / numSamples;
				M2 += delta*((float)l3g_gyro_.z - averageBiasZ);
			}
		}
		else //Calculated for long enough. Print results
		{
			//calculate sigmaZ, the standard deviation of the Z values
			float variance = M2 / (numSamples - 1);
			sigmaZ = sqrt(variance);

			STOP_RATE = sigmaZ / 2;
			Serial.println(STOP_RATE);
			//Print results
			Serial.println(F("Average Bias:"));
			Serial.print(F("Z= "));
			Serial.println(averageBiasZ);

			Serial.println(F("Sigma:"));
			Serial.print(F("Z= "));
			Serial.println(sigmaZ);

			//reset timer and change state
			timer = 0;
			current_state = PrintDirections;
		}
		break;
	case PrintDirections:

		Serial.print("Rotate slowly but steadily clockwise to ");
		Serial.print(ROTATION_ANGLE);
		Serial.println(" and back to initial position.");
		Serial.println("3...");
		delay(300);
		Serial.println("2...");
		delay(300);
		Serial.println("1...");
		delay(300);
		Serial.println("GO");
		hasPrinted = true;
		prev_time = micros();
		current_state = MeasureScaleFactor;
		break;
	case MeasureScaleFactor:
		if(timer == 0) timer = current_time; //Reset timer
		if(!data_avail) return false;
		//measure current rate of rotation
		float rateZ = (float)l3g_gyro_.z - averageBiasZ;
		//Serial.println(rateZ);

		//find angle
		angleZ += (rateZ * (sampleTime / 1000000.0f)); //divide by 1000000.0(convert to sec)

		//Find max angle
		if(angleZ > maxAngleZ) maxAngleZ = angleZ;
		Serial.println(maxAngleZ); //Don't know why, but need this print statement to calibrate.

		//After 3 sec has passed, check if we've stopped at ROTATION_ANGLE
		if(current_time - timer >= 3000000UL)
		{
			//We've stopped turning; we must be at ROTATION_ANGLE
			if(abs(rateZ) < STOP_RATE)
			{
				//Scale factor is the actual amount we rotated the robot divided by the gyro's measured rotation angle
				//Also includes the sensitivity value
				scaleFactor = ROTATION_ANGLE / maxAngleZ;
				Serial.println(F("Done!"));
				Serial.print(F("Max Angle = "));
				Serial.println(maxAngleZ);
				Serial.print(F("Scale Factor = "));
				Serial.println(scaleFactor);

				//Update calibration data
				calibration.averageBiasZ = averageBiasZ;
				calibration.scaleFactorZ = scaleFactor;
				calibration.sigmaZ = sigmaZ;
				//Write calibration data to eeprom
				EEPROM.put(0, calibration);

				//Print out the values put into the eeprom for verification:
				CalibrationData stored_data;
				EEPROM.get(0, stored_data);
				float storedBias = stored_data.averageBiasZ;
				float storedSigma = stored_data.sigmaZ;
				float storedScaleFactor = stored_data.scaleFactorZ;

				Serial.println(F("Printing calculated and stored values..."));
				Serial.print(F("BiasZ = "));
				Serial.print(averageBiasZ);
				Serial.print(F("\tStored = "));
				Serial.println(storedBias);

				Serial.print(F("sigmaZ = "));
				Serial.print(sigmaZ);
				Serial.print(F("\tStored = "));
				Serial.println(storedSigma);

				Serial.print(F("scaleFactorZ = "));
				Serial.print(scaleFactor);
				Serial.print(F("\tStored = "));
				Serial.println(storedScaleFactor);

				return true; //Done calibrating
			}
		}
		break;
	}
	return false;
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