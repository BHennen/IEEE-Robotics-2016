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
 *************************************************************************************************************************
 * program | Switches |                                  Description                                                     *
 *_________|__________|__________________________________________________________________________________________________*
 *    1    |    8     | Competition run. Runs the robot from start to finish. Goes throughout the track to pick up all   *
 *         |          | victims and returns to start.                                                                    *
 *_________|__________|__________________________________________________________________________________________________*
 */

#define usingDipSwitches false //Specify whether or not to use DIP switches to choose program number


// the setup function runs once when you press reset or power the board
void setup() {

}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
