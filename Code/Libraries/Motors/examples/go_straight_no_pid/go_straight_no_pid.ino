////////////////////////////////////////////////////////////////////////////////
// Tells the motors to go "straight" at the same power. (Does not use any PID //
// to ensure they go straight). Straight for 2 sec then stop for 2 sec.       //
////////////////////////////////////////////////////////////////////////////////

#include "Motors.h"
#include "MotorDriver.h"
#include "Sensors.h"
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <L3G.h>
#include <Arduino.h>
#include <EEPROM.h>

Motors *motors;
MotorConfig motor_config = {
	5,	// left_motor_pin_fwd
	6,	// left_motor_pin_bwd
	A0,	// left_motor_current_pin
	9,	// right_motor_pin_fwd
	10,	// right_motor_pin_bwd
	A1,	// right_motor_current_pin
	8,	// enable_pin
	11,	// fault_pin
		//
	10,	// turn_deadzone	//How lenient we want our rotations to be
	100	// drive_power		//power to the drivetrain}
};

// the setup function runs once when you press reset or power the board
void setup()
{
	motors = new Motors(motor_config, nullptr);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	motors->GoUsingPIDControl(0, 0, 0, 0, 0); //Go "straight" (ie no pid control)
	delay(2000); //for 2 sec
	motors->StopMotors();
	delay(2000); //for 2 sec
}
