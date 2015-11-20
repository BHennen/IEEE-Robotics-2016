#include <EEPROMAnything.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <Sensors.h>
Compass* compass;

//TRUE if you are calibrating compass, FALSE if you have already calibrated it, and just want to see the output.
//Calibrating DOES use the EEPROM, which has a limited amount of write/erase cycles. So only calibrate if necessary.
bool calibrate = false;

void setup() 
{
  Serial.begin(9600);
  
  compass = new Compass(calibrate, 0.069522276053);
}

void loop()
{
	if(!calibrate)
	{
		Serial.println(compass->getDegrees());
	}
}
