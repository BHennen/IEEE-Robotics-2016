#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <L3G.h>
#include <Sensors.h>

Gyro *gyro;
byte gyro_interrupt_pin = 2; //Interrupt pin for gyro (on mega, valid choices are 2,3,18,19,20,21)

volatile bool ISR = false;
bool calibrate = true;

void GyroUpdateISR()
{
	gyro->Update();
}

void setup()
{
	Serial.begin(9600);
	/* add setup code here */
	//Enable interrupt on the given pin.
	gyro = new Gyro();
	//while((gyro->l3g_gyro_.readReg(L3G::FIFO_SRC) & B00011111) > 10)
	//{
	//	gyro->l3g_gyro_.read();
	//}
	//attachInterrupt(digitalPinToInterrupt(gyro_interrupt_pin), GyroUpdateISR, RISING);
}

void loop()
{
	gyro->l3g_gyro_.read();
	//if(calibrate && gyro->Calibrate())
	//{
	//	calibrate = false;
	//}
	//else if(!calibrate)
	//{
	//	Serial.println(gyro->GetDegrees());
	//}
}
