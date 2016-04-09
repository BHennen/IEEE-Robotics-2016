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
#include <PixySPI_SS.h>
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
 *         |        1 | victims and returns to start.                                                                    *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog1modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *    2    | oooooo-o | Runs the left motor forward and backward then the right motor forward and backward and loops.    *
 *         |       2  |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog2modules = (MOTORDRIVER);
/*_______________________________________________________________________________________________________________________*
 *    3    | oooooo-- | Tests the Rotate function of Motors class. Runs 4 times clockwise then 4 times counterclockwise  *
 *         |       21 | and loops.                                                                                       *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog3modules = (MOTORDRIVER | MOTORS);
/*_______________________________________________________________________________________________________________________*
 *    4    | ooooo-oo | Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops.        *
 *         |      3   |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog4modules = (GYRO | MOTORDRIVER | MOTORS);
/*_______________________________________________________________________________________________________________________*
 *    5    | ooooo-o- | Tests the GoToVictim function of brain class. Using the Pixy, tries to go to a block using PID   *
 *         |      3 1 | in front.																						 *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog5modules = (VISUALSENSOR | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *    6    | ooooo--0 | Tests the GetBlock function of Sensors class. Print out the block that the GetBlock method       *
 *         |      32  | returns.                                                                                         *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog6modules = (VISUALSENSOR);
/*_______________________________________________________________________________________________________________________*
 *    7    | ooooo--- | Runs a calibration for the Gyro and saves values to EEPROM for future use. Instructions to the   *
 *         |      321 | user are given in serial output.                                                                 *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog7modules = (GYRO | MOTORS | MOTORDRIVER);
/*_______________________________________________________________________________________________________________________*
 *    8    | oooo-ooo | Tests the Gyro. Prints out the angle of the robot to serial.                                     *
 *         |     4    |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog8modules = (GYRO);
/*_______________________________________________________________________________________________________________________*
 *    9    | oooo-oo- | Tests the FollowWall function of Brain class. Follows left wall until Front sensor is too close  *
 *         |     4  1 | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog9modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   10    | oooo-o-o | Tests the FollowWall function of Brain class. Follows right wall until Front sensor is too close *
 *         |     4 2  | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog10modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   11    | oooo-o-- | Tests the FollowWall function of Brain class. Follows left wall until a Gap is detected then     *
 *         |     4 21 | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog11modules = (WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   12    | oooo--oo | Tests the FollowWall function of Brain class. Follows right wall until a Gap is detected then    *
 *         |     43   | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog12modules = (WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   13    | oooo--o- | Tests the FollowWall function of Brain class. Follows left wall until Pixy detects a good block  *
 *         |     43 1 | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog13modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   14    | oooo---o | Tests the FollowWall function of Brain class. Follows right wall until Pixy detects a good block *
 *         |     432  | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog14modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   15    | oooo---- | Tests the GoToLocation function of Brain class. Goes from start to right city victim then stops. *
 *         |     4321 |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog15modules = (VISUALSENSOR | WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
*   16    | ooo-oooo |  Tests the AStarSearch function and BoardState update functions.	Prints to Serial.				 *
*         |    5     |  1)	Print initial board state.                                                            		 *
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
 *         |    5   1 | Doesn't stop.                                                                                    *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog17modules = MOTORS;
/*_______________________________________________________________________________________________________________________*
 *   18    | ooo-oo-o | Tests the wall sensors. Outputs the distance from the desired position on the wall. Doesn't stop.*
 *         |    5  2  |                                                                                                  *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog18modules = WALLSENSORS;
/*_______________________________________________________________________________________________________________________*
 *   19    | ooo-oo-- | Tests the GoStraight function of Motors class. Uses PID control to go straight using ONLY the    *
 *         |    5  21 | encoders for 5 seconds, then prints value of left and right encoder ticks and stops.             *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog19modules = (MOTORS | MOTORDRIVER);
/*_______________________________________________________________________________________________________________________*
 *   20    | ooo-o-oo | Tests the HasVictim function of Sensors class. Prints whether or not it has a victim.			 *
 *         |    5 3   |																						             *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog20modules = (VISUALSENSOR);
/*_______________________________________________________________________________________________________________________*
 *   21    | ooo-o-o- | Tests the FollowWall function of Brain class. Follows left wall forever.						 *
 *         |    5 3 1 |		                                                                                             *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog21modules = (WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   22    | ooo-o--o | Tests the FollowWall function of Brain class. Follows right wall forever.						 *
 *         |    5 32  |		                                                                                             *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog22modules = (WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   23    | ooo-o--- | Tests the TravelPastWall function of Brain class. Goes past the left wall and then stops.		 *
 *         |    5 321 |		                                                                                             *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog23modules = (WALLSENSORS | MOTORDRIVER | MOTORS | BRAIN);
/*_______________________________________________________________________________________________________________________*
 *   24    | ooo--ooo | Tests the TravelPastWall function of Brain class. Goes past the right wall and then stops.		 *
 *         |    54    |		                                                                                             *
 *_________|__________|__________________________________________________________________________________________________*/
const byte prog24modules = (WALLSENSORS  | MOTORDRIVER | MOTORS | BRAIN);
// Choose to use DIP switches or not ////////////////////
#define using_DIP_switches true //Specify whether or not to use DIP switches to choose program number
byte program_number = 8; //Select which program number to use if not using DIP switches

/*** NOTE: DO NOT USE PIN 13 AS DIGITAL INPUT PIN (See arduino reference page) ***/

#if using_DIP_switches
const byte DIP_switch_pins[8] = {37, 36, 38, 40, 42, 43, 49, 48}; //Digital pins for DIP switches. (DIP_switch_pins[0] = switch number 8)
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
	//FIXME: Remove gyro gyro->Update();
}

void LeftEncoderAISR()
{
	motor_driver->UpdateLeftEncoderA();
}

void LeftEncoderBISR()
{
	motor_driver->UpdateLeftEncoderB();
}

void RightEncoderAISR()
{
	motor_driver->UpdateRightEncoderA();
}

void RightEncoderBISR()
{
	motor_driver->UpdateRightEncoderB();
}

ISR(TIMER3_COMPA_vect)
{
	motors->RunPID();
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
		20,	//sensor_gap_min_dist;
		7.0, //desired_dist_to_wall;
		5.0, //min_dist_to_wall;
		7.0, //max_dist_to_wall;
		8.0, //front_sensor_stop_dist;
		15,	 //pixy_block_detection_threshold;
		0.5, //squaring_diff_threshold
		350000, //clearing_time; //How long to go past a gap or wall so we clear the rear end. (microseconds)
		-2.0, //squaring_offset_

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
		 {B10010001,B00000001,B10000001,B10000001,B10000001,B10000001,B10000001,B11000101}},//0
		//     0          1          2          3          4          5          6          7
	};

	MotorConfig motor_config =
	{
		5,		//turn_deadzone; //How lenient we want our rotations to be
		100,	//drive_power; //power to the drivetrain

		7,		//victim_servo_pin

		40,		//victim_servo_closed_angle	0-180
		160,		//victim_servo_open_angle		0-180

		1500000,	//servo_close_time in microsecs
		1500000,	//servo_open_time_ in microsec

		1.25,	//GYRODOMETRY_THRESHOLD Difference in rate between gyro and encoders to use the gyro.

		25000 //PID_sample_time, interval between updating PID values in microseconds NOTE: DO NOT CHANGE UNLESS TIMER UPDATE CHANGES WITH IT
	};

	/*** Constants for the UMBark calibration ***/
#define USE_CALIBRATED_ENCODERS false //define true if we've ran the test and calibrated the encoders. Otherwise we use default values.
	/*** Constants to change ***/
	//Robot constants
	constexpr float b_nominal = 180.0; //Measured length of wheelbase. (mm)
	constexpr float ticks_per_revolution = ((22.0 / 12.0)*(22.0 / 10.0)*(22.0 / 10.0)*(22.0 / 10.0)*(22.0 / 10.0)*(23.0 / 10.0)) * 48;
	constexpr float d_nominal = 140.00; //Measured wheel diameter. (mm)

#if USE_CALIBRATED_ENCODERS //Update the error in wheel diameter and wheelbase
	//Test constants. Run the test then update these values.
	constexpr float L = 4000.0; //Length of square edge. (mm)
	constexpr float L_calc = 3900.0; //Programmed to go L mm, how far did it actually go?
	constexpr float X_cg_cw = -1.0; //X center of gravity clockwise. (mm)
	constexpr float X_cg_ccw = 1.0; //X center of gravity counter clockwise. (mm)

	/*** Constants for math, dont change ***/
	//Calculate adjusted wheel diameter
	constexpr float Es = L_calc / L;
	constexpr float d_adjusted = Es * d_nominal;

	//Calculate actual wheelbase length
	constexpr float alpha = ((X_cg_cw + X_cg_ccw) / (-4.0*L))*(180.0 / M_PI);
	constexpr float b_act = 90.0 / (90.0 - alpha) * b_nominal; //actual wheelbase length

	//Calculate correction factors for left and right wheel diameters
	constexpr float beta = ((X_cg_cw - X_cg_ccw) / (-4.0*L));
	constexpr float R = (L / 2.0) / (sin(beta / 2.0));
	constexpr float Ed = (R + b_act / 2.0) / (R - b_act / 2.0); //Error in wheel diameter
	constexpr float correction_factor_left = 2.0 / (Ed + 1);
	constexpr float correction_factor_right = 2.0 / (1.0 / Ed + 1);
#else//Use default wheel diameter
	constexpr float d_adjusted = d_nominal;
#endif

	//Calculate mms per tick
	constexpr float mms_per_revolution = M_PI * d_adjusted; //PI * diameter
	constexpr float mms_per_tick = mms_per_revolution / ticks_per_revolution;

#if USE_CALIBRATED_ENCODERS
	/*** Calibrated values ***/
	constexpr float left_mms_per_tick = mms_per_tick * correction_factor_left;
	constexpr float right_mms_per_tick = mms_per_tick * correction_factor_right;
	//Wheelbase calculated in math section
#else //Use default values.
	constexpr float left_mms_per_tick = mms_per_tick;
	constexpr float right_mms_per_tick = mms_per_tick;
	constexpr float b_act = b_nominal;
#endif

	byte left_encoder_A_pin = 18; //Interrupt pin (on mega, valid choices are 2,3,18,19,20,21)
	byte left_encoder_B_pin = 19; //Interrupt pin (on mega, valid choices are 2,3,18,19,20,21)
	byte right_encoder_A_pin = 20; //Interrupt pin (on mega, valid choices are 2,3,18,19,20,21)
	byte right_encoder_B_pin = 21; //Interrupt pin (on mega, valid choices are 2,3,18,19,20,21)
	MotorDriverConfig motor_driver_config =
	{
		8,	// left_motor_pin_fwd
		11,	// left_motor_pin_bwd
		A6,	// left_motor_current_pin
		12,	// right_motor_pin_fwd
		13,	// right_motor_pin_bwd
		A7,	// right_motor_current_pin
		29,	// enable_pin
		4,	// fault_pin

		left_encoder_A_pin,	// left_motor_encoder_A
		left_encoder_B_pin,	// left_motor_encoder_B
		right_encoder_A_pin,	// right_motor_encoder_A
		right_encoder_B_pin,	// right_motor_encoder_B

		left_mms_per_tick,	// LEFT_MMS_PER_TICK
		right_mms_per_tick,	// RIGHT_MMS_PER_TICK
		b_act	//	WHEELBASE
	};

	VisualSensorConfig visual_sensor_config =
	{
		47,			//byte pixy_ss; Pin for the slave select of the Pixy
		160,		//center; //Where the robot aims for in PID control. Also affects score of blocks
		{1.0,1.0},	//block_score_consts; //These values are the weights used to determine a blocks score
		100,		//min_block_score;
		15,			//min_block_size;
		80, //byte field_of_view;

		10,	//min_good_bad_ratio; ratio needed for the pixy to successfully confirm a victim is present in its view
		1000000,	//victim_scan_time; how long to scan for victim (microseconds)

		32,		//victim_sensor_pin; //IR receiver
		30,		//victim_emitter_pin; //IR LED
		56000,	//victim_sensor_frequency;
		5000	//ir_scan_time (microseconds)
	};


	WallSensorsConfig wall_sensors_config =
	{
		A8, //front_left_sensor_pin;
		A1, //front_right_sensor_pin;
		A9, //rear_left_sensor_pin;
		A2, //rear_right_sensor_pin;
		A4 //forward_sensor_pin
	};

	byte gyro_interrupt_pin = 2; //Interrupt pin for gyro (on mega, valid choices are 2,3,18,19,20,21)
	GyroConfig gyro_config
	{
		8,//threshold_size; //2<x<31

		//Gyro pins
		53,	//cs 
		50,	//sdo
		51,	//sda
		52	//scl
	};

	RobotConfig robot_config =
	{
		22	//startButtonPin;
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
	case 19:
		modules_to_use = prog19modules;
		break;
	case 20:
		modules_to_use = prog20modules;
		break;
	case 21:
		modules_to_use = prog21modules;
		break;
	case 22:
		modules_to_use = prog22modules;
		break;
	case 23:
		modules_to_use = prog23modules;
		break;
	case 24:
		modules_to_use = prog24modules;
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
		gyro = new Gyro(gyro_config);
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
		//Enable encoder interrupts
		attachInterrupt(digitalPinToInterrupt(left_encoder_A_pin), LeftEncoderAISR, CHANGE);
		attachInterrupt(digitalPinToInterrupt(left_encoder_B_pin), LeftEncoderBISR, CHANGE);
		attachInterrupt(digitalPinToInterrupt(right_encoder_A_pin), RightEncoderAISR, CHANGE);
		attachInterrupt(digitalPinToInterrupt(right_encoder_B_pin), RightEncoderBISR, CHANGE);
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
	if(IEEE_robot->Run())
	{
		while(1);
		//Run with the enabled (or disabled) modules and the selected program.
	}
}