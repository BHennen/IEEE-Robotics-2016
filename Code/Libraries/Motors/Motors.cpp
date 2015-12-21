#include "Motors.h"

/**
* Constructor.
*/
Motors::Motors()
{

}

//Destructor
Motors::~Motors()
{

}

//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
bool Motors::Turn90(Direction d, unsigned long current_time)
{

}

//Uses PID control to go forward, trying to keep the robot aligned with the desired value passed into the function.
void Motors::GoUsingPIDControl(int desired_value, int current_value, int* pid_consts, unsigned long current_time)
{

}

//Goes straight using the gyro or encoders (or both).
void Motors::GoStraight()
{

}