#include <Robot.h>
#include <Sensors.h>
#include <Motors.h>
#include <Brain.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <L3G.h>
#include <MotorDriver.h>

/************************
 * PROGRAM DESCRIPTIONS *
 *************************************************************************************************************************
 * Runs code based on which program number is selected. Programs can be selected by the DIP switches or specified in the *
 * code. If DIP switch program selection is enabled, the program number corresponds to the number on the DIP switches in * 
 * binary. So if switches 6, 7, and 8 are on it will run program 7 (= 1*4 + 1*2 + 1*1).									 *
 *																														 *
 * Below each program are flags for which modules the program requires in order to run. This allows tests to be done     *
 * without requiring all modules to be installed on the robot.															 *
 *************************************************************************************************************************
 *_______________________________________________________________________________________________________________________*
 * program | Switches |                                  Description                                                     *
 *_________|__________|__________________________________________________________________________________________________*
 *    1    | ooooooo- | Competition run. Runs the robot from start to finish. Goes throughout the track to pick up all   *
 *         |        8 | victims and returns to start.                                                                    *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog1modules = VISUALSENSOR | WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *    2    | oooooo-o | Runs the left motor forward and backward then the right motor forward and backward and loops.    *
 *         |       7  |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog2modules = MOTORDRIVER;
/*_______________________________________________________________________________________________________________________*
 *    3    | oooooo-- | Tests the Turn90 function of Motors class. Runs 4 times clockwise then 4 times counterclockwise  *
 *         |       78 | and loops.                                                                                       *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog3modules = GYRO | MOTORDRIVER | MOTORS;
/*_______________________________________________________________________________________________________________________*
 *    4    | ooooo-oo | Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops.        *
 *         |      6   |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog4modules = GYRO | MOTORDRIVER | MOTORS;
/*_______________________________________________________________________________________________________________________*
 *    5    | ooooo-o- | Tests the GoUsingPIDControl function of Motors class. Using the Pixy, tries to go to a block     *
 *         |      6 8 | using PID then stops in front. Will keep going if the block moves.                               *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog5modules = VISUALSENSOR | MOTORDRIVER | MOTORS;
/*_______________________________________________________________________________________________________________________*
 *    6    | ooooo--0 | Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops.        *
 *         |      67  |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog6modules = GYRO | MOTORDRIVER | MOTORS;
/*_______________________________________________________________________________________________________________________*
 *    7    | ooooo--- | Runs a calibration for the Gyro and saves values to EEPROM for future use. Instructions to the   *
 *         |      678 | user are given in serial output.                                                                 *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog7modules = GYRO;
/*_______________________________________________________________________________________________________________________*
 *    8    | oooo-ooo | Tests the Gyro. Prints out the angle of the robot to serial.                                     *
 *         |     5    |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog8modules = GYRO;
/*_______________________________________________________________________________________________________________________*
 *    9    | oooo-oo- | Tests the FollowWall function of Brain class. Follows left wall until Front sensor is too close  *
 *         |     5  8 | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog9modules = VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   10    | oooo-o-o | Tests the FollowWall function of Brain class. Follows right wall until Front sensor is too close *
 *         |     5 7  | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog10modules = VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   11    | oooo-o-- | Tests the FollowWall function of Brain class. Follows left wall until a Gap is detected then     *
 *         |     5 78 | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog11modules = WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   12    | oooo--oo | Tests the FollowWall function of Brain class. Follows right wall until a Gap is detected then    *
 *         |     56   | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog12modules = WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   13    | oooo--o- | Tests the FollowWall function of Brain class. Follows left wall until Pixy detects a good block  *
 *         |     56 8 | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog13modules = VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   14    | oooo---o | Tests the FollowWall function of Brain class. Follows right wall until Pixy detects a good block *
 *         |     567  | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog14modules = VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   15    | oooo---- | Tests the GoAtoB function of Brain class. Goes from start to crossroad then stops.               *
 *         |     5678 |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog15modules = VISUALSENSOR | WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN;

// Choose to use DIP switches or not ////////////////////
#define using_DIP_switches false //Specify whether or not to use DIP switches to choose program number
byte program_number = 2; //Select which program number to use if not using DIP switches

#if using_DIP_switches
const byte DIP_switch_pins[8] = {1, 2, 3, 4, 5, 6, 7, 8}; //Digital pins for DIP switches. (DIP_switch_pins[0] = switch number 1)
#endif

// Set up pointers to modules for the robot and the robot itself. ////////////////////
Brain* brain;
Motors* motors;
MotorDriver *motor_driver;
VisualSensor *visual_sensor;
WallSensors *wall_sensors;
Gyro *gyro;
Robot *IEEE_robot;

RobotModules robot_modules =
{
	brain,			//Brain 
	visual_sensor,	//VisualSensor
	wall_sensors,	//WallSensors
	gyro,			//Gyro 
	motors,			//Motors 
	motor_driver	//MotorDriver 
};

BrainModules brain_modules =
{
	visual_sensor,	//VisualSensor
	wall_sensors,	//WallSensors
	motors,			//Motors 
	gyro			//Gyro 
};

// Configuration data for the modules. Edit at will. ////////////////////////////////////
BrainConfig brain_config =
{
	//Variables for wall following
	10,	//sensor_gap_min_dist;
	5,	//desired_dist_to_wall;
	3,	//front_sensor_stop_dist;
	10	//pixy_block_detection_threshold;
};

MotorConfig motor_config = 
{
	5,		//turn_deadzone; //How lenient we want our rotations to be
	100		//drive_power; //power to the drivetrain
};

MotorDriverConfig motor_driver_config = 
{
	5,	// left_motor_pin_fwd
	6,	// left_motor_pin_bwd
	A0,	// left_motor_current_pin
	9,	// right_motor_pin_fwd
	10,	// right_motor_pin_bwd
	A1,	// right_motor_current_pin
	8,	// enable_pin
	11	// fault_pin
};

VisualSensorConfig visual_sensor_config =
{
	5,			//ir_port;
	160,		//center; //Where the robot aims for in PID control. Also affects score of blocks
	{1.0,1.0},	//block_score_consts; //These values are the weights used to determine a blocks score
	100,		//min_block_score;
	15			//min_block_size;
};

WallSensorsConfig wall_sensors_config = 
{
	1, //front_left_sensor_pin
	2, //front_right_sensor_pin
	3, //rear_left_sensor_pin
	4, //rear_right_sensor_pin
};

RobotConfig robot_config =
{
	69	//startButtonPin;
};

// the setup function runs once when you press reset or power the board
void setup()
{
//Read dip switch pins (if enabled) to change program number
#if using_DIP_switches
	program_number = 0;
	for(int pin_num = 1; pin_num <= 8; pin_num++)
	{
		//set pinmode of dip switch to input
		pinMode(DIP_switch_pins[pin_num - 1], INPUT);

		//if dipswitch is on, convert the pin from binary number and add that to program number
		//if pin 6 and 8 are on, for example, this would yield 5 (4 + 1).
		program_number += digitalRead(DIP_switch_pins[pin_num - 1]) ? pow(2, (8 - pin_num)) : 0;
	}
#endif

	Serial.begin(9600);

	//Set up the modules to use for our robot, depending on the program selected
	byte modules_to_use;
	switch(program_number)
	{
		case 1:
			modules_to_use = prog1modules;
			break;
		case 2:
			modules_to_use = prog2modules;
			break;
		case 3:
			modules_to_use = prog3modules;
			break;
		case 4:
			modules_to_use = prog4modules;
			break;
		case 5:
			modules_to_use = prog5modules;
			break;
		case 6:
			modules_to_use = prog6modules;
			break;
		case 7:
			modules_to_use = prog7modules;
			break;
		case 8:
			modules_to_use = prog8modules;
			break;
		case 9:
			modules_to_use = prog9modules;
			break;
		case 10:
			modules_to_use = prog10modules;
			break;
		case 11:
			modules_to_use = prog11modules;
			break;
		case 12:
			modules_to_use = prog12modules;
			break;
		case 13:
			modules_to_use = prog13modules;
			break;
		case 14:
			modules_to_use = prog14modules;
			break;
		case 15:
			modules_to_use = prog15modules;
			break;
		default:
			Serial.print("ERROR- Invalid program choice: ");
			Serial.println(program_number);
			while(1);
			break;
	}

	//Construct new modules depending on what we need for the program. Otherwise, leave it as nullptr
	if(modules_to_use & BRAIN)
	{
		brain = new Brain(brain_modules, brain_config);
	}
	else
	{
		brain = nullptr;
	}

	if(modules_to_use & MOTORS)
	{
		motors = new Motors(motor_config, gyro, motor_driver);
	}
	else
	{
		motors = nullptr;
	}

	if(modules_to_use & MOTORDRIVER)
	{
		motor_driver = new MotorDriver(motor_driver_config);
	}
	else
	{
		motor_driver = nullptr;
	}

	if(modules_to_use & VISUALSENSOR)
	{
		visual_sensor = new VisualSensor(visual_sensor_config);
	}
	else
	{
		visual_sensor = nullptr;
	}

	if(modules_to_use & WALLSENSORS)
	{
		wall_sensors = new WallSensors(wall_sensors_config);
	}
	else
	{
		wall_sensors = nullptr;
	}
	
	if(modules_to_use & GYRO)
	{
		gyro = new Gyro();
	}
	else
	{
		gyro = nullptr;
	}

	IEEE_robot = new Robot(program_number, robot_config, robot_modules);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	IEEE_robot->Run();
}
