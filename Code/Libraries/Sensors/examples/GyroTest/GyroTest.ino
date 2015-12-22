#include <Sensors.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <eeprom.h>
#include <L3G.h>

Gyro* gyro;

void setup()
{
	Serial.begin(9600);
	gyro = new Gyro();
}

void loop()
{
	unsigned long current_time = millis();
	gyro->Update(current_time);
	Serial.print("Angle = ");
	Serial.println(gyro->GetDegrees());
}
