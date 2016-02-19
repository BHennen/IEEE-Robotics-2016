#include <Arduino.h>
#include <pnew.cpp>
#include <Bitset.h>
#include <iterator>
#include <States.h>
#include "Directions.h"
#include <Robot.h>
#include <Sensors.h>
#include <Servo.h>
#include <Motors.h>
#include <BrainEnums.h>
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
const byte prog1modules = (VISUALSENSOR | WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *    2    | oooooo-o | Runs the left motor forward and backward then the right motor forward and backward and loops.    *
 *         |       7  |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog2modules = (MOTORDRIVER);
/*_______________________________________________________________________________________________________________________*
 *    3    | oooooo-- | Tests the Turn90 function of Motors class. Runs 4 times clockwise then 4 times counterclockwise  *
 *         |       78 | and loops.                                                                                       *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog3modules = (GYRO | MOTORDRIVER | MOTORS);
/*_______________________________________________________________________________________________________________________*
 *    4    | ooooo-oo | Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops.        *
 *         |      6   |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog4modules = (GYRO | MOTORDRIVER | MOTORS);
/*_______________________________________________________________________________________________________________________*
 *    5    | ooooo-o- | Tests the GoToVictim function of Brain class which indirectly tests GoUsingPIDControl function.  *
 *         |      6 8 | Using the Pixy, tries to go to a block using PID then stops in front. Returns true when stopped. *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog5modules = (VISUALSENSOR | MOTORDRIVER | MOTORS);
/*_______________________________________________________________________________________________________________________*
 *    6    | ooooo--0 | Tests the GetBlock function of Sensors class. Print out the block that the GetBlock method       *
 *         |      67  | returns.                                                                                         *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog6modules = (VISUALSENSOR);
/*_______________________________________________________________________________________________________________________*
 *    7    | ooooo--- | Runs a calibration for the Gyro and saves values to EEPROM for future use. Instructions to the   *
 *         |      678 | user are given in serial output.                                                                 *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog7modules = (GYRO);
/*_______________________________________________________________________________________________________________________*
 *    8    | oooo-ooo | Tests the Gyro. Prints out the angle of the robot to serial.                                     *
 *         |     5    |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog8modules = (GYRO);
/*_______________________________________________________________________________________________________________________*
 *    9    | oooo-oo- | Tests the FollowWall function of Brain class. Follows left wall until Front sensor is too close  *
 *         |     5  8 | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog9modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   10    | oooo-o-o | Tests the FollowWall function of Brain class. Follows right wall until Front sensor is too close *
 *         |     5 7  | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog10modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   11    | oooo-o-- | Tests the FollowWall function of Brain class. Follows left wall until a Gap is detected then     *
 *         |     5 78 | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog11modules = (WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   12    | oooo--oo | Tests the FollowWall function of Brain class. Follows right wall until a Gap is detected then    *
 *         |     56   | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog12modules = (WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   13    | oooo--o- | Tests the FollowWall function of Brain class. Follows left wall until Pixy detects a good block  *
 *         |     56 8 | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog13modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   14    | oooo---o | Tests the FollowWall function of Brain class. Follows right wall until Pixy detects a good block *
 *         |     567  | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog14modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   15    | oooo---- | Tests the GoToLocation function of Brain class. Goes from start to right city victim then stops. *
 *         |     5678 |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog15modules = (VISUALSENSOR | WALLSENSORS | GYRO | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
*   16    | ooo-oooo |  Tests the AStarSearch function and BoardState update functions.	Prints to Serial.				 *
*         |    4     |  1)	Print initial board state.                                                            		 *
*         |          |  2)	Print path to right city victim.                                              				 *
*         |          |  3)	Remove right city victim, update robot state. Print board state.                      		 *
*         |          |  4)	Print path to red victim drop off, update robot state.                        				 *
*         |          |  5)	Print path to lower right grass victim. 													 *
*         |          |	6)	Update lower right grass victim location to northern spot. Print board state.    			 *
*         |          |	7)	Update left grass location to southern spot. Print board state.								 *
*         |          |	8)	Update robot to lower right corner and remove right grass victim. Print new board state.	 *
*         |          |	9)	Print path to upper left victim (longest feasible path).									 *
*_________|__________|___________________________________________________________________________________________________*/
const byte prog16modules = BRAIN;
/*_______________________________________________________________________________________________________________________*
 *   17    | ooo-ooo- | Tests the BiteVictim and ReleaseVictim functions of Motor class. Closes on victim then opens.    *
 *         |    4   8 | Doesn't stop.                                                                                    *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog17modules = MOTORS;
/*_______________________________________________________________________________________________________________________*
 *   18    | ooo-oo-o | Tests the wall sensors. Outputs the distance from the desired position on the wall. Doesn't stop.*
 *         |    4  7  |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog18modules = WALLSENSORS;
// Choose to use DIP switches or not ////////////////////
#define using_DIP_switches true //Specify whether or not to use DIP switches to choose program number
byte program_number = 8; //Select which program number to use if not using DIP switches

/*** NOTE: DO NOT USE PIN 13 AS DIGITAL INPUT PIN (See arduino reference page) ***/

#if using_DIP_switches
const byte DIP_switch_pins[8] = {22, 23, 24, 25, 26, 27, 28, 29}; //Digital pins for DIP switches. (DIP_switch_pins[0] = switch number 1)
#endif

// Set up pointers to modules for the robot and the robot itself. ////////////////////
Brain* brain;
Motors* motors;
MotorDriver *motor_driver;
VisualSensor *visual_sensor;
WallSensors *wall_sensors;
Gyro *gyro;
Robot *IEEE_robot;

//Interrupt Service Routine to update the gyro's raw angle whenever the data is ready
void GyroUpdateISR()
{
	gyro->Update();
}

void setup()
{
	Serial.begin(9600);
	delay(1000);
	//Read dip switch pins (if enabled) to change program number
#if using_DIP_switches
	program_number = 0;
	for(int pin_num = 1; pin_num <= 8; pin_num++)
	{
		//set pinmode of dip switch to input_pullup
		pinMode(DIP_switch_pins[pin_num - 1], INPUT_PULLUP);

		//if dipswitch is on, convert the pin from binary number and add that to program number
		//if pin 6 and 8 are on, for example, this would yield 5 (4 + 1).
		int num_read = pow(2, (8 - pin_num)) + 0.5; //+0.5 because of rounding issues
		//inverted because of input_pullup. (Default is high when switch is off)
		bool pin_enabled = !digitalRead(DIP_switch_pins[pin_num - 1]);
		Serial.println(pin_enabled);
		program_number += pin_enabled ? num_read : 0;
	}
	Serial.print(F("Program: "));
	Serial.println(program_number);
#endif
	
	// Configuration data for the modules. Edit at will. ////////////////////////////////////
	//individual grid point for storing info about the board
	BrainConfig brain_config =
	{
		//Variables for wall following
		10,	//sensor_gap_min_dist;
		5,	//desired_dist_to_wall;
		3,	//front_sensor_stop_dist;
		10,	//pixy_block_detection_threshold;
	
		//State configuration
		RIGHT,				  //Direction init_direction
		static_cast<byte>(0), //byte init_x
		static_cast<byte>(0),  //byte init_y

		//Board configuration
		//Each cell is a binary BXXXX number:
		//		B76543210
		//		where:
		//		7 = north wall
		//		6 = east wall
		//		5 = south wall
		//		4 = west wall
		//		3 = yellow dropoff
		//		2 = red dropoff
		//		1 = victim location
		//		0 = passable
		//
		{{B10010001,B10000001,B10000001,B10000001,B10000001,B10000011,B10000001,B11000001},//7
		 {B00010001,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01000001},//6
		 {B00010011,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01000011},//5
		 {B00010000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B01000001},//4
		 {B00110011,B00100001,B00100001,B00000001,B00000001,B00000001,B00000001,B01100001},//3
		 {B10110010,B10100001,B10100001,B00000001,B00100001,B00100001,B00000001,B11100001},//2
		 {B10111001,B10000001,B10100001,B00100001,B10100001,B10100001,B00100001,B11100010},//1
		 {B10110001,B00100001,B10100001,B10100001,B10100001,B10100001,B10100001,B11100101}},//0
		//     0          1          2          3          4          5          6          7
	};
	
	MotorConfig motor_config = 
	{
		5,		//turn_deadzone; //How lenient we want our rotations to be
		100,	//drive_power; //power to the drivetrain

		50,		//victim_servo_pin
		//41,	//right_servo_pin

		60,		//victim_servo_closed_angle	0-180
		//90,	//right_servo_closed_angle	0-180
		140,		//victim_servo_open_angle		0-180
		//90,	//right_servo_open_angle	0-180

		1000000,	//servo_close_time in microsecs
		1000000		//servo_open_time_ in microsec
	};
	
	MotorDriverConfig motor_driver_config = 
	{
		7,	// left_motor_pin_fwd
		6,	// left_motor_pin_bwd
		A1,	// left_motor_current_pin
		4,	// right_motor_pin_fwd
		5,	// right_motor_pin_bwd
		A0,	// right_motor_current_pin
		12,	// enable_pin
		11	// fault_pin
	};
	
	VisualSensorConfig visual_sensor_config =
	{
		5,			//ir_port;
		160,		//center; //Where the robot aims for in PID control. Also affects score of blocks
		{1.0,1.0},	//block_score_consts; //These values are the weights used to determine a blocks score
		100,		//min_block_score;
		15,			//min_block_size;

		100,	//min_good_bad_ratio; ratio needed for the pixy to successfully confirm a victim is present in its view
		1000000,	//victim_scan_time; how long to scan for victim (microseconds)

		16	//victim_sensor_pin
	};
	
	byte gyro_interrupt_pin = 2; //Interrupt pin for gyro (on mega, valid choices are 2,3,18,19,20,21)
	byte gyro_threshold_size = 8; //How many gyro readings to store before we update the angle. (2 < threshold < 32)

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
		case 16:
			modules_to_use = prog16modules;
			break;
		case 17:
			modules_to_use = prog17modules;
			break;
		case 18:
			modules_to_use = prog18modules;
			break;
		default:
			Serial.print(F("ERROR- Invalid program choice: "));
			Serial.println(program_number);
			while(1);
			break;
	}
	//Serial.println(modules_to_use);
	//Construct new modules depending on what we need for the program. Otherwise, leave it as nullptr
	if(modules_to_use & GYRO)
	{
		gyro = new Gyro(gyro_threshold_size);
		//Enable interrupt on the given pin.
		attachInterrupt(digitalPinToInterrupt(gyro_interrupt_pin), GyroUpdateISR, RISING);
	}
	else
	{
		gyro = nullptr;
	}
	
	if(modules_to_use & MOTORDRIVER)
	{
		motor_driver = new MotorDriver(motor_driver_config);
	}
	else
	{
		motor_driver = nullptr;
	}

	if(modules_to_use & MOTORS)
	{
		motors = new Motors(motor_config, gyro, motor_driver);
	}
	else
	{
		motors = nullptr;
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

	if(modules_to_use & BRAIN)
	{
		BrainModules brain_modules =
		{
			visual_sensor,	//VisualSensor
			wall_sensors,	//WallSensors
			motors,			//Motors 
			gyro			//Gyro 
		};
		brain = new Brain(brain_modules, brain_config);
	}
	else
	{
		brain = nullptr;
	}

	RobotModules robot_modules =
	{
		brain,			//Brain 
		visual_sensor,	//VisualSensor
		wall_sensors,	//WallSensors
		gyro,			//Gyro 
		motors,			//Motors 
		motor_driver	//MotorDriver 
	};
	IEEE_robot = new Robot(program_number, robot_config, robot_modules);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	IEEE_robot->Run(); //Run with the enabled (or disabled) modules and the selected program.
}