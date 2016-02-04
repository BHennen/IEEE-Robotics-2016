#include "Robot.h"

Robot::Robot(byte program_number, RobotConfig robot_config, RobotModules robot_modules)
{
	brain_ = robot_modules.brain;
	visual_sensor_ = robot_modules.visual_sensor;
	wall_sensors_ = robot_modules.wall_sensors;
	gyro_ = robot_modules.gyro;
	motors_ = robot_modules.motors;
	drivetrain_ = robot_modules.drivetrain;

	config = robot_config;
	program_ = program_number;

	pinMode(config.startButtonPin, INPUT_PULLUP); //configure startButtonPin to be pulled high when nothing is connected

	completed = false;
}

Robot::~Robot()
{

}

/**
 * Runs the robot, based on whichever program was selected. Only runs if the start button isnt connected or is pressed.
 * Returns true when it has completed said program.
 */
bool Robot::Run()
{
	//Read startButtonPin. If nothing is connected digitalRead will return HIGH (because of INPUT_PULLUP), otherwise if the 
	//switch is connected and pressed digital read will return HIGH, when not pressed will return LOW.
	if(digitalRead(config.startButtonPin))
	{
		//Only run if we haven't completed the program yet.
		if(!completed)
		{
			//At the beginning of the loop, if we have a gyro update the angle.
			if(gyro_)
			{
				gyro_->Update();
			}

			//Run the program selected when the robot powers on.
			switch(program_)
			{
			case 1:
				completed = FinalRun();
				break;
			case 2:
				completed = TestMotorsDemo();
				break;
			case 3:
				completed = TestMotorsTurn90();
				break;
			case 4:
				completed = TestMotorsFollowHeading();
				break;
			case 5:
				completed = TestGoToVictim();
				break;
			case 6:
				completed = TestPixyGetBlock();
				break;
			case 7:
				completed = CalibrateGyro();
				break;
			case 8:
				completed = TestGyroOutput();
				break;
			case 9:
				completed = TestBrainFollowWallFront(LEFT);
				break;
			case 10:
				completed = TestBrainFollowWallFront(RIGHT);
				break;
			case 11:
				completed = TestBrainFollowWallGap(LEFT);
				break;
			case 12:
				completed = TestBrainFollowWallGap(RIGHT);
				break;
			case 13:
				completed = TestBrainFollowWallPixy(LEFT);
				break;
			case 14:
				completed = TestBrainFollowWallPixy(RIGHT);
				break;
			case 15:
				completed = TestGoToLocation();
				break;
			case 16:
				completed = TestAStarSearch();
				break;
			default:
				Serial.print(F("ERROR- Invalid program choice: "));
				Serial.println(program_);
				completed = true;
				break;
			}
		}
	}
	return completed;
}

// Test Programs //////////////////////////////////////

/**
* Program: 1
* Competition run. Runs the robot from start to finish. Goes throughout the track to pick up all victims and returns to start.
* Returns true when it has returned to start.
*/
bool Robot::FinalRun()
{
	auto GoToDropoffLocation = [this]() -> bool
	{
		ActionResult result;
		if(brain_->victim_sig == 1) //yellow victim
		{
			result = brain_->GoToLocation(0, 1);
		}
		else //red victim
		{
			result = brain_->GoToLocation(7, 0);
		}
		switch(result)
		{
		case ACT_GOING:
			return false;
			break;
		case ACT_SUCCESS://we're at dropoff location, return true
			return true;
			break;
		default: //Error or fail stop code (shouldn't happen when going to first victim, so return true to stop robot)
			return true;
			break;
		}
	};

	switch(brain_->num_victims)
	{
	case 0: //Pick up and drop off right city victim.
		//If brain doesn't have the victim in its grasp
		if(!brain_->has_victim)
		{
			//Go to victims location if we haven't already moved there
			if(!brain_->done_moving)
			{
				switch(brain_->GoToLocation(7, 1))
				{
				case ACT_GOING: // do nothing
					break;
				case ACT_SUCCESS://we're on top of the victim; move to next step
					brain_->done_moving = true;
					break;
				default: //Error or fail stop code (shouldn't happen when going to first victim, so return true to stop robot)
					return true;
					break;
				}
			}
			else //we're done moving, pick up victim
			{
				//bite victim
				if(motors_->BiteVictim())
				{
					//when done biting, 
					brain_->board_state_.RemoveVictim(7, 1); //remove victim from board state
					brain_->done_moving = false; //say we're not done moving
					brain_->has_victim = true; //signal that we have a victim
				}
			}
		}
		else //robot has victim in its grasp
		{
			//Go to drop off location if we haven't already moved there
			if(!brain_->done_moving)
			{
				if(GoToDropoffLocation())
				{
					brain_->done_moving = true;
				}
			}
			else //we're done moving, drop off victim
			{
				if(motors_->ReleaseVictim())
				{
					//when done biting, signal that we dropped off a victim
					brain_->done_moving = false;
					brain_->has_victim = false;
					brain_->num_victims++; //now dropped off 1 more victim; go to next one
				}
			}
		}
		break;
	case 1://Pick up and drop off left city victim
		static bool checking_mexico = true;
		//instead of immediately going to the victim, we check the grass area to see if there is a victim
		//and update the board state, then go to the left city victim.
		if(checking_mexico)
		{
			//Go to scan location and face left side of board
			if(!brain_->done_moving)
			{
				switch(brain_->GoToLocation(3, 3, LEFT))
				{
				case ACT_GOING: // do nothing
					break;
				case ACT_SUCCESS://we're at the scan location; move to next step
					brain_->done_moving = true;
					break;
				default: //Error or fail stop code (shouldn't happen when going to second victim, so return true to stop robot)
					return true;
					break;
				}
			}
			else //we're done moving, scan for the victim
			{
				//scan
				switch(visual_sensor_->ScanForVictim())
				{
				case 0://results negative, victim not in the south; victim is up
					brain_->board_state_.SetLeftVictimLocation(UP);
					brain_->done_moving = false;
					checking_mexico = false;
					break;
				case 1://results positive, victim in the south
					brain_->board_state_.SetLeftVictimLocation(DOWN);
					brain_->done_moving = false;
					checking_mexico = false;
					break;
				case 2://still scanning
					break;
				}
			}
		}
		//Done checking for victim, go to left city victim. If brain doesn't have the victim in its grasp
		else if(!brain_->has_victim)
		{
			//Go to victims location if we haven't already moved there
			if(!brain_->done_moving)
			{
				switch(brain_->GoToLocation(0, 2))
				{
				case ACT_GOING: // do nothing
					break;
				case ACT_SUCCESS://we're on top of the victim; move to next step
					brain_->done_moving = true;
					break;
				default: //Error or fail stop code (shouldn't happen when going to second victim, so return true to stop robot)
					return true;
					break;
				}
			}
			else //we're done moving, pick up victim
			{
				//bite victim
				if(motors_->BiteVictim())
				{
					//when done biting, 
					brain_->board_state_.RemoveVictim(0, 2); //remove victim from board state
					brain_->done_moving = false; //say we're not done moving
					brain_->has_victim = true; //signal that we have a victim
				}
			}
		}
		else //robot has victim in its grasp
		{
			//Go to drop off location if we haven't already moved there
			if(!brain_->done_moving)
			{
				if(GoToDropoffLocation())
				{
					brain_->done_moving = true;
				}
			}
			else //we're done moving, drop off victim
			{
				if(motors_->ReleaseVictim())
				{
					//when done biting, signal that we dropped off a victim
					brain_->done_moving = false;
					brain_->has_victim = false;
					brain_->num_victims++; //now dropped off 1 more victim; go to next one
				}
			}
		}
		break;
	case 2: //Go to right grass victim (to ensure we can get to the left victim if it is behind the river)
		//If brain doesn't have the victim in its grasp
		if(!brain_->has_victim)
		{
			static bool go_to_alt_location = false;
			//Go to victims location if we haven't already moved there
			if(!brain_->done_moving)
			{
				ActionResult result_flags = go_to_alt_location ? brain_->GoToLocation(5, 7) : brain_->GoToLocation(7, 5);
				//Check if we've encountered a wall before we encounter the victim
				//If true, this means we need to go to the alternate location instead.
				if(result_flags & ACT_FRONT)
				{
					//Update our location on the board
					brain_->robot_state_ = RobotState(UP, 7, 7);
					//update the victim's location on the board.
					brain_->board_state_.SetRightVictimLocation(UP);
					//signal that next loop we go to the alternate path
					go_to_alt_location = true;
					return false;
				}
				switch(result_flags)
				{
				case ACT_GOING: // do nothing
					break;
				case ACT_SUCCESS://we're on top of the victim; move to next step
					//If we got here without using alternate path, update the victim's location on the board
					if(!go_to_alt_location) brain_->board_state_.SetRightVictimLocation(DOWN);
					brain_->done_moving = true;
					break;
				default: //Error or fail stop code (shouldn't happen when going to first victim, so return true to stop robot)
					return true;
					break;
				}
			}
			else //we're done moving, pick up victim
			{
				//bite victim
				if(motors_->BiteVictim())
				{
					//when done biting, 
					if(go_to_alt_location)
					{
						brain_->board_state_.RemoveVictim(5, 7); //remove victim from board state
					}
					else
					{
						brain_->board_state_.RemoveVictim(7, 5); //remove victim from board state
					}
					brain_->done_moving = false; //say we're not done moving
					brain_->has_victim = true; //signal that we have a victim
				}
			}
		}
		else //robot has victim in its grasp
		{
			//Go to drop off location if we haven't already moved there
			if(!brain_->done_moving)
			{
				if(GoToDropoffLocation())
				{
					brain_->done_moving = true;
				}
			}
			else //we're done moving, drop off victim
			{
				if(motors_->ReleaseVictim())
				{
					//when done biting, signal that we dropped off a victim
					brain_->done_moving = false;
					brain_->has_victim = false;
					brain_->num_victims++; //now dropped off 1 more victim; go to next one
				}
			}
		}
		break;
	case 3://Go to left grass victim
		//If brain doesn't have the victim in its grasp
		if(!brain_->has_victim)
		{
			bool go_to_mexico = brain_->board_state_.HasVictim(0, 3);
			//Go to victims location if we haven't already moved there
			if(!brain_->done_moving)
			{
				ActionResult result = go_to_mexico ? brain_->GoToLocation(0, 3) : brain_->GoToLocation(0, 5);
				switch(result)
				{
				case ACT_GOING: // do nothing
					break;
				case ACT_SUCCESS://we're on top of the victim; move to next step
					brain_->done_moving = true;
					break;
				default: //Error or fail stop code (shouldn't happen when going to first victim, so return true to stop robot)
					return true;
					break;
				}
			}
			else //we're done moving, pick up victim
			{
				//bite victim
				if(motors_->BiteVictim())
				{
					//when done biting,
					if(go_to_mexico)
					{
						brain_->board_state_.RemoveVictim(0, 3); //remove victim from board state
					}
					else
					{
						brain_->board_state_.RemoveVictim(0, 5); //remove victim from board state
					}					
					brain_->done_moving = false; //say we're not done moving
					brain_->has_victim = true; //signal that we have a victim
				}
			}
		}
		else //robot has victim in its grasp
		{
			//Go to drop off location if we haven't already moved there
			if(!brain_->done_moving)
			{
				if(GoToDropoffLocation())
				{
					brain_->done_moving = true;
				}
			}
			else //we're done moving, drop off victim
			{
				if(motors_->ReleaseVictim())
				{
					//when done biting, signal that we dropped off a victim
					brain_->done_moving = false;
					brain_->has_victim = false;
					brain_->num_victims++; //now dropped off 1 more victim; go to next one
				}
			}
		}
		break;
	case 4://We've deposited all 4 victims, return to start
		switch(brain_->GoToLocation(0, 0))
		{
		case ACT_GOING: // do nothing
			break;
		case ACT_SUCCESS://we're back at start, return true
			return true;
			break;
		}
		break;
	}
	return false;
}

/**
 * Program: 2
 * Runs the left motor forward and backward then the right motor forward and backward and loops. Always returns false.
 */
bool Robot::TestMotorsDemo()
{
	//Lambda function to stop motors if drivetrain encounters a fault.
	auto StopIfFault = [this]()
	{
		if(drivetrain_->isFault())
		{
			Serial.println(F("motor driver fault"));
			while(1);
		}
	};

	//Lambda function to print the current current draw of a motor
	auto PrintCurrent = [this](int i, bool left)
	{
		if(abs(i) % 50 == 0)
		{
			if(left)
			{
				Serial.print(F("Left Motor current: "));
				Serial.println(drivetrain_->GetLeftCurrentMilliamps());
			}
			else
			{
				Serial.print(F("Right Motor current: "));
				Serial.println(drivetrain_->GetRightCurrentMilliamps());
			}
		}
	};

	//Lambda function run a motor at a speed and print current
	auto RunMotor = [StopIfFault, PrintCurrent, this](int speed, bool left)
	{
		StopIfFault();
		if(left)
		{
			drivetrain_->SetLeftSpeed(speed);
		}
		else
		{
			drivetrain_->SetRightSpeed(speed);
		}
		PrintCurrent(speed, left);
	};

	//Run left motor up to max, down and to reverse max, and back to 0
	for(int i = 0; i <= 255; i++)
	{
		RunMotor(i, true);
		delay(15);
	}

	for(int i = 255; i >= -255; i--)
	{
		RunMotor(i, true);
		delay(15);
	}

	for(int i = -255; i <= 0; i++)
	{
		RunMotor(i, true);
		delay(15);
	}

	//Run right motor up to max, down and to reverse max, and back to 0
	for(int i = 0; i <= 255; i++)
	{
		RunMotor(i, false);
		delay(15);
	}

	for(int i = 255; i >= -255; i--)
	{
		RunMotor(i, false);
		delay(15);
	}

	for(int i = -255; i <= 0; i++)
	{
		RunMotor(i, false);
		delay(15);
	}
	return false;
}

/**
 * Program: 3
 * Tests the Turn90 function of Motors class. Runs 4 times clockwise then 4 times counterclockwise and loops. Always returns false.
 */
bool Robot::TestMotorsTurn90()
{
	static Direction rotate_dir = RIGHT;
	static byte rot_num = 0;

	//Turn 90 in rotate direction
	if(motors_->Turn90(rotate_dir))
	{
		rot_num++; //increment number of times we've rotated in that direction if we've completed the rotation
	}
	//If we've rotated 4 times in desired direction, reset rotation counts and switch direction.
	if(rot_num == 4)
	{
		rot_num = 0;
		rotate_dir = (rotate_dir == RIGHT) ? LEFT : RIGHT;
	}

	return false;
}

/**
 * Program: 4
 * Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops and returns true.
 */
bool Robot::TestMotorsFollowHeading()
{
	//Get heading (wherever it was pointing when function was first called)
	static float desired_heading = gyro_->GetDegrees();
	static bool reset_PID = true;

	//Reset PID once before using FollowHeading (which uses the PID control function)
	if(reset_PID)
	{
		motors_->ResetPID();
		reset_PID = false;
	}

	if(motors_->FollowHeading(desired_heading, 5000000UL))
	{
		reset_PID = true; //Follow heading completed; reset PID for next time.
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Program: 5
 * Tests the GoUsingPIDControl function of Motors class. Using the Pixy, tries to go to a block using PID then stops
 * in front. Will keep going if the block moves. Always returns false.
 */
bool Robot::TestGoToVictim()
{
	return brain_->GoToVictim();
}

/**
 * Program: 6
 * Tests the GetBlock function of Sensors class. Print out the block that the GetBlock method returns.
 */
bool Robot::TestPixyGetBlock()
{
	Block block = visual_sensor_->GetBlock();

	if(block.signature == visual_sensor_->BAD_BLOCK.signature)
	{
		Serial.println(F("GetBlock() returned BAD_BLOCK."));
	}
	else
	{
		block.print();
	}

	return false;
}

/**
 * Program: 7
 * Runs a calibration for the Gyro and saves values to EEPROM for future use. Instructions to the user are given
 * in serial output. Returns true when calibration is completed.
 */
bool Robot::CalibrateGyro()
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
	const float STOP_RATE = 0.1f; //How slow to be considered stopped
	const unsigned long averagingTime = 5000UL; //Optimal time based on allan variance is 5 sec

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

	switch(current_state)
	{
	case CalculateBias:
		//Check if we have calculated bias for the desired amount of time.
		if(timer == 0) timer = current_time;
		if(current_time - timer < averagingTime)
		{
			//Keep a running average of current value of the gyro
			numSamples++;
			float delta = (float)gyro_->l3g_gyro_.g.z - averageBiasZ;
			averageBiasZ += delta / numSamples;
			M2 += delta*((float)gyro_->l3g_gyro_.g.z - averageBiasZ);
		}
		else //Calculated for long enough. Print results
		{
			//calculate sigmaZ, the standard deviation of the Z values
			float variance = M2 / (numSamples - 1);
			sigmaZ = sqrt(variance);

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

		current_state = MeasureScaleFactor;
		break;
	case MeasureScaleFactor:
		if(timer == 0) timer = current_time; //Reset timer

		unsigned long sampleTime = current_time - timer;

		//measure current rate of rotation
		float rateZ = ((float)gyro_->l3g_gyro_.g.z - averageBiasZ) * READING_TO_DPS;

		//find angle
		angleZ += (rateZ * sampleTime / 1000000.0f); //divide by 1000000.0(convert to sec)

		//Find max angle
		if(angleZ > maxAngleZ) maxAngleZ = angleZ;

		//After 3 sec has passed, check if we've stopped at ROTATION_ANGLE (or if hasRotated, starting position)
		if(current_time - timer >= 3000UL)
		{
			//We've stopped turning; we must be at ROTATION_ANGLE (or if hasRotated, starting position)
			if(abs(rateZ) < STOP_RATE)
			{
				//If we havent rotated to ROTATION_ANGLE yet
				if(!hasRotated)
				{
					Serial.println(F("Rotate back to starting position."));
					timer = 0; //Reset timer
					hasRotated = true;
				}
				else //we're back where we started. Measure scale factor. Save bias and scale factor to eeprom, then output current readings.
				{
					//Scale factor is the actual amount we rotated the robot divided by the gyro's measured rotation angle
					//Also includes the sensitivity value
					scaleFactor = ROTATION_ANGLE / maxAngleZ * READING_TO_DPS;
					Serial.println(F("Done!"));
					Serial.print(F("Max Angle = "));
					Serial.println(maxAngleZ);
					Serial.print(F("Scale Factor = "));
					Serial.println(scaleFactor);

					//Update calibration data
					CalibrationData calibration_data;
					calibration_data.averageBiasZ = averageBiasZ;
					calibration_data.scaleFactorZ = scaleFactor;
					calibration_data.sigmaZ = sigmaZ;
					//Write calibration data to eeprom
					EEPROM.put(0, calibration_data);

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
		}
		break;
	}
	return false;
}

/**
 * Program: 8
 * Tests the Gyro. Prints out the angle of the robot to serial. Always returns false.
 */
bool Robot::TestGyroOutput()
{
	Serial.println(gyro_->GetDegrees());
	return false;
}

/**
 * Program: 9, 10
 * Tests the FollowWall function of Brain class. Follows wall until Front sensor is too close then stops and returns true.
 */
bool Robot::TestBrainFollowWallFront(Direction dir)
{
	//Follow wall until FRONT stop condition is met.
	if(brain_->FollowWall(dir, StopConditions::FRONT) == StopConditions::FRONT)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Program: 11, 12
 * Tests the FollowWall function of Brain class. Follows wall until a Gap is detected then stops and returns true.
 */
bool Robot::TestBrainFollowWallGap(Direction dir)
{
	//Follow wall until FRONT stop condition is met.
	if(brain_->FollowWall(dir, StopConditions::GAP) == StopConditions::GAP)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Program: 13, 14
 * Tests the FollowWall function of Brain class. Follows wall until Pixy detects a good block then stops and returns true.
 */
bool Robot::TestBrainFollowWallPixy(Direction dir)
{
	//Follow wall until FRONT stop condition is met.
	if(brain_->FollowWall(dir, StopConditions::PIXY) == StopConditions::PIXY)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Program: 15
 * Tests the GoAtoB function of Brain class. Goes from start to crossroad then stops and returns true.
 */
bool Robot::TestGoToLocation()
{
	switch(brain_->GoToLocation(static_cast <byte>(3), static_cast <byte>(1)))
	{
	case ACT_GOING:
		return false;
		break;
	case ACT_SUCCESS:
		return true;
		break;
	default://print error and return true
		Serial.println(F("Error: GoToLocation returned something not expected for this test run."));
		return true;
		break;
	}
	return false;
}

/**
* Program: 16
* Tests the AStarSearch function. Prints out a path from start to the first victim on right.
* Expected results:
* FOLLOW LEFT success: GAP  fail: NONE
* ROTATE LEFT
* TPW LEFT
* ROTATE RIGHT
* TPW RIGHT
* FOLLOW RIGHT success: PIXY  fail: NONE
* GO TO VICTIM
*/
bool Robot::TestAStarSearch()
{
	//Loop through resulting byte_action_list from A* results and print each action
	for(byte_action action : SearchAlgorithm::AStarGoAToB(7, 1, brain_->robot_state_, brain_->board_state_))
	{
		SearchAlgorithm::PrintByteActionString(action);
	}
	return true;
}
