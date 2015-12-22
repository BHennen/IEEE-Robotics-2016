#include <SPI.h>
#include <Pixy.h>
#include <Sensors.h>
#include <Wire.h>
#include <eeprom.h>
#include <l3g.h>

/**************************************
 * Test Sketch for VisualSensor class *
 **************************************
 * Tests all of the functions of the VisualSensor class.
 * Should print out to serial:
 * [stuff to test here]
 */

VisualSensor *sensors;

void setup() 
{
  Serial.begin(9600);
  //sensors = new VisualSensor();
}

void loop() 
{
  //Test the functions of the visual sensor here
  
}
