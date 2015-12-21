#ifndef Motors_H
#define Motors_H

#include <Arduino.h>

enum Direction
{
	left,
	right
};

/**
 * Class that contains Motors to act on the world.
 */
class Motors
{
public:
	/**
	 * Constructor. 
	 */
	Motors();
		
	//Destructor
	~Motors();

	//Turns the robot in a direction d until it reaches 90 degrees, then returns true. Uses gyro or encoders (or both).
	bool Turn90(Direction d, unsigned long current_time);

	//Uses PID control to go forward, trying to keep the robot aligned with the desired value passed into the function.
	void GoUsingPIDControl(int desired_value, int current_value, int* pid_consts, unsigned long current_time);
	
	//Goes straight using the gyro or encoders (or both).
	void GoStraight();

private:
	
};

#endif