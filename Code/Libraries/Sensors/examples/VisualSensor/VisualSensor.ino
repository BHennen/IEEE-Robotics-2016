#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>
#include <Wire.h>
#include <eeprom.h>
#include <EEPROMAnything.h>
#include <l3g.h>

/**************************************
 * Test Sketch for VisualSensor class *
 **************************************
 * Tests all of the functions of the VisualSensor class.
 * Should print out to serial:
 * 1) If the pixy sees a block: "Detected Block: [block information]"
 * 2) If the pixy doesn't see a block: "No block"
 * 3) If the IR sensor is close to an oject: "IR sensor is close"
 */

VisualSensor *sensors;

const char IRPort = A0; //Port for IR sensor
float stopVoltage = 2.8; //Voltage to stop the robot

void setup() 
{
  Serial.begin(9600);
  sensors = new VisualSensor(IRPort, stopVoltage);
}

void loop() 
{
  //Test the block finding method; print the lowest block found in the Pixy's view
  Block block = (*sensors).getBlock();
  if(block.signature != (*sensors).badBlock.signature) //Check if the detected signature is not equal to the badBlock's signature. If it's not, it is a good block.
  {
    Serial.print("Detected block:");
    block.print();
  }
  else
  {
    Serial.println("No block");
  }
  
  //Test the isClose(); see if the IR sensor is working.
  if((*sensors).isClose())
  {
    Serial.println("IR sensor is close");  
  }
}
