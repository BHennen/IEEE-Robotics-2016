#ifndef Robot_H
#define Robot_H

#include "Arduino.h"
#include "Brain.h"
#include "Sensors.h"
#include "Motors.h"

struct RobotConfig
{
	byte startButtonPin;
};

struct RobotModules
{
	Brain *brain;
	VisualSensor *visual_sensor;
	WallSensors *wall_sensors;
	Gyro *gyro;
	Motors *motors;
	MotorDriver *drivetrain;
};

enum UsedModules
{
	NONE = 0,
	VISUALSENSOR = 1 << 0,
	WALLSENSORS = 1 << 1,
	GYRO = 1 << 2,
	MOTORDRIVER = 1 << 3,
	MOTORS  = 1 << 4,
	BRAIN = 1 << 5
};

class Robot
{
public:	
	// Variables //////////////////////////////////////////
	
	RobotConfig config;
	bool completed;

	// Functions //////////////////////////////////////////
	Robot(byte program_number, RobotConfig robot_config, RobotModules robot_modules);

	~Robot();

	bool Run();

private:
	// Variables //////////////////////////////////////////
	Brain *brain_;
	VisualSensor *visual_sensor_;
	WallSensors *wall_sensors_;
	Gyro *gyro_;
	Motors *motors_;
	MotorDriver *drivetrain_;

	byte program_;

	// Functions //////////////////////////////////////////
	//1
	bool FinalRun();

	//2
	bool TestMotorsDemo();

	//3
	bool TestMotorsTurn90();

	//4
	bool TestMotorsFollowHeading();

	//5
	bool TestGoToVictim();

	//6
	bool TestPixyGetBlock();

	//7
	bool CalibrateGyro();
	
	//8
	bool TestGyroOutput();

	//9,10
	bool TestBrainFollowWallFront(Direction dir);

	//11,12
	bool TestBrainFollowWallGap(Direction dir);

	//13,14
	bool TestBrainFollowWallPixy(Direction dir);

	//15
	bool TestBrainGoStartToXRoad();

	//16
	bool TestAStarSearch();
};


#endif