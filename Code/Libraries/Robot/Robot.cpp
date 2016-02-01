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
					completed = TestMotorsPidPixy();
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
					completed = TestBrainGoStartToXRoad();
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
bool Robot::TestMotorsPidPixy()
{
	return false;
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
	return gyro_->Calibrate();
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
bool Robot::TestBrainGoStartToXRoad()
{
	return false;
}
