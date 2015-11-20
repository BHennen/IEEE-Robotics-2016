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
	int nextAddress = EEPROM_readAnything(0, averageBiasZ);
	//read sigmaZ
	nextAddress += EEPROM_readAnything(nextAddress, sigmaZ);
	//read scaleFactor
	nextAddress += EEPROM_readAnything(nextAddress, scaleFactorZ);

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


/**
* Constructor. Set the initial heading whenever the program starts.
* -declinationAngle: 'Error' of the magnetic field in your location. Find yours here: http://www.magnetic-declination.com/.
* -(for Norman, I calculated it to be: 0.069522276053)
*/
Compass::Compass(bool calibrate, float declinationAngle)
{
	/* Initialize the sensor */
	if (!begin())
	{
		/* There was a problem detecting the HMC5883 ... check your connections */
		Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
	}

	_hmc5883_Gauss_LSB_XY = 1100.0F;  // Varies with gain
	_hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

	_calibrate = calibrate;
	_declinationAngle = declinationAngle;

	if (_calibrate)
	{
		_centerPoint.x = 0.0;
		_centerPoint.y = 0.0;
		_centerPoint.z = 0.0;

		this->calibrate();
	}

	//Read values from eeprom
	int nextAddress = 0;
	nextAddress = EEPROM_readAnything(nextAddress, _centerPoint) + 1;
	EEPROM_readAnything(nextAddress, _rotationMatrix);
	_initMagVector = getMagVector(true);

}

/**
* Deconstructor
*/
Compass::~Compass()
{

}

void printVect(float v[3])
{
	Serial.print("X=");	Serial.print(v[0]);
	Serial.print(" Y="); Serial.print(v[1]);
	Serial.print(" Z="); Serial.println(v[2]);
}

void printMatrix(float m[3][3])
{
	Serial.print("|"); Serial.print(m[0][0]); Serial.print(" "); Serial.print(m[0][1]); Serial.print(" "); Serial.print(m[0][2]); Serial.println("|");
	Serial.print("|"); Serial.print(m[1][0]); Serial.print(" "); Serial.print(m[1][1]); Serial.print(" "); Serial.print(m[1][2]); Serial.println("|");
	Serial.print("|"); Serial.print(m[2][0]); Serial.print(" "); Serial.print(m[2][1]); Serial.print(" "); Serial.print(m[2][2]); Serial.println("|");
}

//Return distance between two points
float distance(hmc5883MagData p1, hmc5883MagData p2)
{
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}

/**
* Rotates a given point (inPoint) about a given a rotation matrix (matrix) and stores the result in outPoint
* Has the option to calculate the rotation of Z by setting calculateZ to true
*/
void rotate(float inPoint[3], float matrix[3][3], float outPoint[3], bool calculateZ)
{
	outPoint[0] = matrix[0][0] * inPoint[0] + matrix[0][1] * inPoint[1] + matrix[0][2] * inPoint[2];
	outPoint[1] = matrix[1][0] * inPoint[0] + matrix[1][1] * inPoint[1] + matrix[1][2] * inPoint[2];
	if (calculateZ)
	{
		outPoint[2] = matrix[2][0] * inPoint[0] + matrix[2][1] * inPoint[1] + matrix[2][2] * inPoint[2];
	}
}

/**
* Multiply 2 3x3 matrices (inMatrix1 and inMatrix2) and stores the result in outMatrix
*/
void matrixMultiply3x3(float inMatrix1[3][3], float inMatrix2[3][3], float outMatrix[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			outMatrix[i][j] = 0;
			for (int k = 0; k < 3; k++)
			{
				outMatrix[i][j] += inMatrix1[i][k] * inMatrix2[k][j];
			}
		}
	}
}

/**
* Multiply a matrix times a scalar
*/
void matrixTimesScalar(float inMatrix[3][3], float scalar, float outMatrix[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			outMatrix[i][j] = inMatrix[i][j] * scalar;
		}
	}
}

/**
* Add 2 3x3 matrices inMatrix1 and inMatrix2 and stores the result in outMatrix
*/
void addMatrices(float inMatrix1[3][3], float inMatrix2[3][3], float outMatrix[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			outMatrix[i][j] = inMatrix1[i][j] + inMatrix2[i][j];
		}
	}
}

/**
* Finds the rotation matrix between two points (p1 and p2) and stores the result in outRotationMatrix
* Uses rodrigues roation formula. Set axis to {0,0,0} if you dont care what axis(axes) the points are rotated about.
*/
void findRotationMatrix(float p1[3], float p2[3], float outRotationMatrix[3][3], float axis[3])
{
	//Calculate magnitude of desired axis
	float magAxis = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
	float rotAxis[3] = { axis[0], axis[1], axis[2] };
	//If magAxis is not 0, we want to rotate about a certain axis. Make sure it is normalized.
	if (magAxis != 0)
	{
		rotAxis[0] /= magAxis;
		rotAxis[1] /= magAxis;
		rotAxis[2] /= magAxis;
	}

	//Calculate magnitude of the two points
	float magP1 = sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]);
	float magP2 = sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2]);

	//Calculate normalized vectors a and b.
	float a[3] = {
		(p1[0] / magP1),
		(p1[1] / magP1),
		(p1[2] / magP1) };
	float b[3] = {
		(p2[0] / magP2),
		(p2[1] / magP2),
		(p2[2] / magP2) };
	//Subtract from them the normalized desired axis of rotation
	a[0] *= (1 - rotAxis[0]);
	a[1] *= (1 - rotAxis[1]);
	a[2] *= (1 - rotAxis[2]);
	b[0] *= (1 - rotAxis[0]);
	b[1] *= (1 - rotAxis[1]);
	b[2] *= (1 - rotAxis[2]);
	//Renormalize vectors
	float magA = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	float magB = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
	a[0] /= magA;
	a[1] /= magA;
	a[2] /= magA;
	b[0] /= magB;
	b[1] /= magB;
	b[2] /= magB;

	//calculate cross product of a and b (a x b = v)
	float v[3] = {
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]
	};

	//Sine of angle (magnitude of v)
	float s = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	//cosine of angle (dot product of a and b)
	float c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	//Calculate the axis of rotation 
	float vNorm[3];
	if (magAxis == 0)
	{//If no rotation axis was given,
		//calculate cross product normalized
		if (s != 0)
		{//only if the cross product exists, to prevent division by 0
			vNorm[0] = v[0] / s;
			vNorm[1] = v[1] / s;
			vNorm[2] = v[2] / s;
		}
		else
		{//v is 0
			vNorm[0] = v[0];
			vNorm[1] = v[1];
			vNorm[2] = v[2];
		}
	}
	else
	{//Set axis of rotation to be the given normalized axis. The sign of the axis is determined by the sign of the cross product
		vNorm[0] = (v[0] > 0) ? rotAxis[0] : -rotAxis[0];
		vNorm[1] = (v[1] > 0) ? rotAxis[1] : -rotAxis[1];
		vNorm[2] = (v[2] > 0) ? rotAxis[2] : -rotAxis[2];
	}

	//Identity Matrix = I
	float identity[3][3] = {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 }
	};
	//cosine times I = cI
	float cI[3][3];
	matrixTimesScalar(identity, c, cI);

	//skew-symmetric cross-product matrix = K
	float K[3][3] = {
		{ 0, -vNorm[2], vNorm[1] },
		{ vNorm[2], 0, -vNorm[0] },
		{ -vNorm[1], vNorm[0], 0 }
	};
	//sine times K = sK 
	float sK[3][3];
	matrixTimesScalar(K, s, sK);

	//Tensor product = T
	float T[3][3] = {
		{ vNorm[0] * vNorm[0], vNorm[0] * vNorm[1], vNorm[0] * vNorm[2] },
		{ vNorm[0] * vNorm[1], vNorm[1] * vNorm[1], vNorm[1] * vNorm[2] },
		{ vNorm[0] * vNorm[2], vNorm[1] * vNorm[2], vNorm[2] * vNorm[2] },
	};
	//1 - cosine times T = (1 - c)*T
	float constTimesT[3][3];
	matrixTimesScalar(T, (1.0f - c), constTimesT);

	// cI + sK
	float cIPlussK[3][3];
	addMatrices(cI, sK, cIPlussK);

	//Rotation Matrix = (cI + sK) + (1 - c)*T
	addMatrices(cIPlussK, constTimesT, outRotationMatrix);

}

void Compass::calibrate()
{
	//Rotate the robot slowly in a circle (direction does not matter)
	//Calculate the center.x, center.y, and center.z by averaging all the values


	Serial.println("Start rotating the robot slowly when you see GO.");
	delay(1000);
	Serial.println("3...");
	delay(1000);
	Serial.println("2...");
	delay(1000);
	Serial.println("1...");
	delay(1000);
	Serial.println("GO");

	unsigned long timer = millis();
	hmc5883MagData startPoint = getMagVector(false);
	hmc5883MagData currentPoint = startPoint;
	hmc5883MagData prevPoint = startPoint;
	float distToPrevPoint = 999.0;

	int numPoints = 1;
	float xTotal = 0.0;
	float yTotal = 0.0;
	float zTotal = 0.0;

	hmc5883MagData southish = startPoint;
	float maxDistToStart = 0.0;
	hmc5883MagData westish = startPoint;
	float bestEquidistance = 0.0;

	while (_calibrate)
	{
		unsigned long currentTime = millis();
		float currDistToStart = distance(currentPoint, startPoint);

		//Only add value of the current point when it is sufficiently far away from the previous point. 
		//This will make the calibration data more uniform if there are pauses during rotation.
		if (distToPrevPoint > 1)
		{
			//Find the total x, y and z values of all the points
			xTotal += currentPoint.x;
			yTotal += currentPoint.y;
			zTotal += currentPoint.z;

			//Find the furthest point(southish) from the start point		
			if (currDistToStart > maxDistToStart)
			{
				maxDistToStart = currDistToStart;
				southish = currentPoint;
			}

			//Find the most equidistant point between the current point to start point and current point to southish point
			float currEquidistance = currDistToStart + distance(currentPoint, southish);
			if (currEquidistance > bestEquidistance)
			{
				bestEquidistance = currEquidistance;
				westish = currentPoint;
			}
			//Update prevpoint and numPoints since the current point was used in a calculation
			numPoints++;
			prevPoint = currentPoint;
		}

		//Get the next mag vector for the current point
		currentPoint = getMagVector(false);
		distToPrevPoint = distance(currentPoint, prevPoint);

		//When at least 5 seconds have passed...
		if (currentTime - timer >= 5000UL)
		{
			//...and the current point is close to the starting point, end the calibration. Calculate and save the center point to the EEPROM
			if (currDistToStart < 1)
			{
				//Find the center point
				_centerPoint.x = xTotal / numPoints;
				_centerPoint.y = yTotal / numPoints;
				_centerPoint.z = zTotal / numPoints;

				//Translate startPoint and westish point coordinates based on the new center
				startPoint.x -= _centerPoint.x;
				startPoint.y -= _centerPoint.y;
				startPoint.z -= _centerPoint.z;
				westish.x -= _centerPoint.x;
				westish.y -= _centerPoint.y;
				westish.z -= _centerPoint.z;
				//Convert to float[3] vectors
				float startPointVector[3] = { startPoint.x, startPoint.y, startPoint.z };
				float westishVector[3] = { westish.x, westish.y, westish.z };
				//Print the vectors' initial values
				Serial.println("StartPoint Vector Initially: ");
				printVect(startPointVector);
				Serial.println("Westish Vector Initially: ");
				printVect(westishVector);

				//Find rotation matrix from the start point to the positive Y axis (north).
				//After this rotation, startPoint should be on the +Y axis. {0,>0,0}
				float rotMatrixNorth[3][3];
				float northVector[3] = { 0, 1, 0 };
				float rotAxisAny[3] = { 0.0f, 0.0f, 0.0f }; //Don't care about rotation axis
				findRotationMatrix(startPointVector, northVector, rotMatrixNorth, rotAxisAny);
				//Print north vector after rotation to see if the rotation matrix was calculated successfully
				float startPointOnY[3];
				rotate(startPointVector, rotMatrixNorth, startPointOnY, true);
				Serial.println("StartPoint Vector after rotation to \"North\" {0,>0,0} : ");
				printVect(startPointOnY);

				//Rotate the westish point based on the north rotation matrix and store it in westishRot1
				float westishRot1[3];
				rotate(westishVector, rotMatrixNorth, westishRot1, true);
				//Find rotation matrix from westishRot1 to the negative X axis (West) on the XY plane
				//After this rotation, westishRot1 should be: {<0,Q,0} where Q is close to 0
				float westVector[3] = { -1, westishRot1[1], 0 };
				float rotMatrixWest[3][3];
				float rotAxisY[3] = { 0, 1, 0 }; //Rotate about Y axis
				findRotationMatrix(westishRot1, westVector, rotMatrixWest, rotAxisY);
				//Print westish vector after rotation to see if the rotation matrix was calculated successfully
				float westishPointOnXY[3];
				rotate(westishRot1, rotMatrixWest, westishPointOnXY, true);
				Serial.println("Westish Vector after rotation to \"West\" {<0,Q,0} where Q is close to 0 : ");
				printVect(westishPointOnXY);

				//Calculate a single, combined rotation matrix of north rotation and west rotation and store it as a single rotation matrix
				//Composing matrices done in reverse order
				matrixMultiply3x3(rotMatrixWest, rotMatrixNorth, _rotationMatrix);
				//Print north and westish vector after combined rotation to see if the combined rotation matrix was calculated successfully
				float startPointOnYcombined[3];
				float westishPointOnXYcombined[3];
				rotate(startPointVector, _rotationMatrix, startPointOnYcombined, true);
				Serial.println("StartPoint Vector after combined rotation {0,>0,0} : ");
				printVect(startPointOnYcombined);
				rotate(westishVector, _rotationMatrix, westishPointOnXYcombined, true);
				Serial.println("Westish Vector after combined rotation {<0,Q,0} where Q is close to 0 : ");
				printVect(westishPointOnXYcombined);

				//Write the center point to the eeprom
				int nextAddress = 0;
				nextAddress = EEPROM_writeAnything(nextAddress, _centerPoint) + 1;
				//Write the rotation matrix to eeprom
				EEPROM_writeAnything(nextAddress, _rotationMatrix);

				Serial.println("Data saved successfully!");

				_calibrate = false;
			}
		}
	}
}

/**
 * Returns the vector of magnetic values that the magnetometor is currently reading.
 * Bool adjusted determines whether or not to return the values adjusted for the center and rotation matrix obtained during calibration.
 */
hmc5883MagData Compass::getMagVector(bool adjusted)
{
	// Read the magnetometer
	Wire.beginTransmission((byte)HMC5883_ADDRESS_MAG);
	Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
	Wire.endTransmission();
	Wire.requestFrom((byte)HMC5883_ADDRESS_MAG, (byte)6);

	// Wait around until enough data is available
	while (Wire.available() < 6);

	// Note high before low (different than accel)  
	uint8_t xhi = Wire.read();
	uint8_t xlo = Wire.read();
	uint8_t zhi = Wire.read();
	uint8_t zlo = Wire.read();
	uint8_t yhi = Wire.read();
	uint8_t ylo = Wire.read();

	// Shift values to create properly formed integer (low byte first)
	hmc5883MagData magData;
	magData.x = (int16_t)(xlo | ((int16_t)xhi << 8));
	magData.y = (int16_t)(ylo | ((int16_t)yhi << 8));
	magData.z = (int16_t)(zlo | ((int16_t)zhi << 8));

	// Convert values to correct numbers
	hmc5883MagData magVector;
	magVector.x = magData.x / _hmc5883_Gauss_LSB_XY * 100; //* 100 to convert from gauss to microtesla
	magVector.y = magData.y / _hmc5883_Gauss_LSB_XY * 100;
	magVector.z = magData.z / _hmc5883_Gauss_LSB_Z * 100;

	if (adjusted)
	{
		//Translate the values so they are centered
		magVector.x -= _centerPoint.x;
		magVector.y -= _centerPoint.y;
		magVector.z -= _centerPoint.z;

	}

	return magVector;
}

/**
* Returns the how many degrees the robot is rotated from the initial heading. Always positive, and always less than 180 degrees. (0 <= degrees < 180)
*/
float Compass::getDegrees()
{
	hmc5883MagData magVector = getMagVector(true); //The magnetic values stored in a vector
	float vectArray[3] = { magVector.x, magVector.y, magVector.z };
	float vectRot[3];
	rotate(vectArray, _rotationMatrix, vectRot, false);

	//Calculate current rotation
	float heading = atan2(vectRot[1], vectRot[0]);

	// Correct for when signs are reversed.
	if (heading < 0)
		heading += 2 * PI;

	// Check for wrap due to addition of declination.
	if (heading > 2 * PI)
		heading -= 2 * PI;

	// Convert radians to degrees.
	float headingDegrees = heading * 180 / PI;
	return headingDegrees;
}

/**
 * Set the magnetometer's gain
 */
void Compass::setGain(hmc5883MagGain gain)
{
	write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte)gain);

	switch (gain)
	{
	case HMC5883_MAGGAIN_1_3:
		_hmc5883_Gauss_LSB_XY = 1100;
		_hmc5883_Gauss_LSB_Z = 980;
		break;
	case HMC5883_MAGGAIN_1_9:
		_hmc5883_Gauss_LSB_XY = 855;
		_hmc5883_Gauss_LSB_Z = 760;
		break;
	case HMC5883_MAGGAIN_2_5:
		_hmc5883_Gauss_LSB_XY = 670;
		_hmc5883_Gauss_LSB_Z = 600;
		break;
	case HMC5883_MAGGAIN_4_0:
		_hmc5883_Gauss_LSB_XY = 450;
		_hmc5883_Gauss_LSB_Z = 400;
		break;
	case HMC5883_MAGGAIN_4_7:
		_hmc5883_Gauss_LSB_XY = 400;
		_hmc5883_Gauss_LSB_Z = 255;
		break;
	case HMC5883_MAGGAIN_5_6:
		_hmc5883_Gauss_LSB_XY = 330;
		_hmc5883_Gauss_LSB_Z = 295;
		break;
	case HMC5883_MAGGAIN_8_1:
		_hmc5883_Gauss_LSB_XY = 230;
		_hmc5883_Gauss_LSB_Z = 205;
		break;
	}
}

/**
* Set up the magnetometer
*/
bool Compass::begin()
{
	// Enable I2C
	Wire.begin();

	// Enable the magnetometer
	write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

	// Set the gain to a known level
	setGain(HMC5883_MAGGAIN_1_3);

	return true;
}

/**
* Write data to the magnetometer
*/
void Compass::write8(byte address, byte reg, byte value)
{
	Wire.beginTransmission(address);
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t)value);
	Wire.endTransmission();
}