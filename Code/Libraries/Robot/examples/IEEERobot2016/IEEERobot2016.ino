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
 *    2    |    7     | Runs the left motor forward and backward then the right motor forward and backward and loops.    *
 *_________|__________|__________________________________________________________________________________________________*
 *    3    |   7, 8   | Tests the Turn90 function of Motors class. Runs 4 times clockwise then 4 times counterclockwise  *
 *         |          | and loops.                                                                                       *
 *_________|__________|__________________________________________________________________________________________________*
 *    4    |    6     | Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops.        *
 *_________|__________|__________________________________________________________________________________________________*
 *    5    |   6, 8   | Tests the GoUsingPIDControl function of Motors class. Using the Pixy, tries to go to a block     *
 *         |          | using PID then stops in front. Will keep going if the block moves.                               *
 *_________|__________|__________________________________________________________________________________________________*
 *    6    |   6, 7   | Tests the FollowHeading function of Motors class. Goes straight for 5 seconds then stops.        *
 *_________|__________|__________________________________________________________________________________________________*
 *    7    | 6, 7, 8  | Runs a calibration for the Gyro and saves values to EEPROM for future use. Instructions to the   *
 *         |          | user are given in serial output.                                                                 *
 *_________|__________|__________________________________________________________________________________________________*
 *    8    |    5     | Tests the Gyro. Prints out the angle of the robot to serial.                                     *
 *_________|__________|__________________________________________________________________________________________________*
 *    9    |   5, 8   | Tests the FollowWall function of Brain class. Follows wall until Front sensor is too close       *
 *         |          | then stops.                                                                                      *
 *_________|__________|__________________________________________________________________________________________________*
 *   10    |   5, 7   | Tests the FollowWall function of Brain class. Follows wall until a Gap is detected then stops.   *
 *_________|__________|__________________________________________________________________________________________________*
 *   11    | 5, 7, 8  | Tests the FollowWall function of Brain class. Follows wall until Pixy detects a good block then  *
 *         |          | stops.                                                                                           *
 *_________|__________|__________________________________________________________________________________________________*
 *   12    |   5, 6   | Tests the GoAtoB function of Brain class. Goes from start to frontier then stops.                *
 *_________|__________|__________________________________________________________________________________________________*
 */

#define usingDipSwitches false //Specify whether or not to use DIP switches to choose program number


// the setup function runs once when you press reset or power the board
void setup() {

}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
