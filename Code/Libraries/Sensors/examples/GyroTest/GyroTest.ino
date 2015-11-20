#include <Sensors.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <eeprom.h>
#include <EEPROMAnything.h>
#include <L3G.h>

Gyro* gyro;
/*** GYRO ***/
const float gyrokp = 4.0f;	//proportional
const float gyroki = 0.5f;	//integral
const float gyrokd = 0.5f;	//derivative
float gyroPIDconsts[3] = { gyrokp, gyroki, gyrokd };

void setup()
{
	Serial.begin(9600);
	gyro = new Gyro(gyroPIDconsts,gyroPIDconsts);
}

void loop()
{
	unsigned long currentTime = millis();
	gyro->update(currentTime);
	Serial.print("Angle = ");
	Serial.println(gyro->getDegrees());
}
