#include "Robot.h"

Robot::Robot(byte program, RobotModules robot_modules)
{
	
}

Robot::~Robot()
{

}

/**
 * Runs the robot, based on whichever program was selected. Only runs if the start button is pressed. 
 * Returns true when it has completed said program.
 */
bool Robot::Run()
{
	return false;
}

// Test Programs //////////////////////////////////////

/**
 * Runs the left motor forward and backward then the right motor forward and backward and loops. Always returns false.
 */
bool Robot::TestMotorsDemo()
{
	return false;
}

/**
 * Tests the Turn90 function of Motors class. Runs 4 times clockwise then 4 times counterclockwise and loops. Always returns false.
 */
bool Robot::TestMotorsTurn90()
{
	return false;
}

/**
 * Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops and returns true.
 */
bool Robot::TestMotorsFollowHeading()
{
	return false;
}

/**
 * Tests the GoUsingPIDControl function of Motors class. Using the Pixy, tries to go to a block using PID then stops
 * in front. Will keep going if the block moves. Always returns false.
 */
bool Robot::TestMotorsPidPixy()
{
	return false;
}

/**
 * Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops and returns true.
 */
bool Robot::TestPixyGetBlock()
{
	return false;
}

/**
 * Runs a calibration for the Gyro and saves values to EEPROM for future use. Instructions to the user are given 
 * in serial output. Returns true when calibration is completed.
 */
bool Robot::CalibrateGyro()
{
	return false;
}

/**
 * Tests the Gyro. Prints out the angle of the robot to serial. Always returns false.
 */
bool Robot::TestGyroOutput()
{
	return false;
}

/**
 * Tests the FollowWall function of Brain class. Follows wall until Front sensor is too close then stops and returns true.
 */
bool Robot::TestBrainFollowWallFront()
{
	return false;
}

/**
 * Tests the FollowWall function of Brain class. Follows wall until a Gap is detected then stops and returns true.
 */
bool Robot::TestBrainFollowWallGap()
{
	return false;
}

/**
 * Tests the FollowWall function of Brain class. Follows wall until Pixy detects a good block then stops and returns true.
 */
bool Robot::TestBrainFollowWallPixy()
{
	return false;
}

/**
 * Tests the GoAtoB function of Brain class. Goes from start to frontier then stops and returns true.
 */
bool Robot::TestBrainGoStartToFrontier()
{
	return false;
}

/**
 * Competition run. Runs the robot from start to finish. Goes throughout the track to pick up all victims and returns to start.
 * Returns true when it has returned to start.
 */
bool Robot::FinalRun()
{
	return false;
}
