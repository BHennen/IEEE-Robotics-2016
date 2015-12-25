//////////////////////////////////////////////////
// Test an IR sensor connected to a desired pin //
//////////////////////////////////////////////////
#include <Arduino.h>
#include <EEPROM.h>
#include "Sensors.h"
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <L3G.h>

WallSensorsConfig config = 
{
	A0,//front_left_sensor_pin;
	A1,//front_right_sensor_pin;
	A2,//rear_left_sensor_pin;
	A3//rear_right_sensor_pin;
};
SensorPosition pos_to_read = REAR_RIGHT;
WallSensors *sensors;

// the setup function runs once when you press reset or power the board
void setup()
{
	Serial.begin(9600);
	sensors = new WallSensors(config);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	Serial.println(sensors->ReadSensor(pos_to_read));
}
