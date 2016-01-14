#ifndef Robot_H
#define Robot_H

#include "Arduino.h"
#include "Brain.h"
#include "Sensors.h"
#include "Motors.h"

struct RobotConfig
{
	byte program;
	byte startButtonPin;
};

struct RobotModules
{
	Brain *brain;
	VisualSensor *visual_sensor;
	WallSensors *wall_sensors;
	Gyro *gyro;
	Motors *motors;
};

class Robot
{
public:	
	// Variables //////////////////////////////////////////
	
	RobotConfig config;
	bool completed;

	// Functions //////////////////////////////////////////
	Robot(RobotConfig robot_config, RobotModules robot_modules);

	~Robot();

	bool Run();

private:
	// Variables //////////////////////////////////////////
	Brain *brain_;
	VisualSensor *visual_sensor_;
	WallSensors *wall_sensors_;
	Gyro *gyro_;
	Motors *motors_;

	byte program_;

	// Functions //////////////////////////////////////////
	bool FinalRun();

	bool TestMotorsDemo();

	bool TestMotorsTurn90();

	bool TestMotorsFollowHeading();

	bool TestMotorsPidPixy();

	bool TestPixyGetBlock();

	bool CalibrateGyro();
	
	bool TestGyroOutput();

	bool TestBrainFollowWallFront();

	bool TestBrainFollowWallGap();

	bool TestBrainFollowWallPixy();

	bool TestBrainGoStartToFrontier();
};


#endif