/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#include <Wire.h>
#include <L3G.h>

L3G gyro;
int desiredNumTimeSlots = 10;
int AllanNumber = 1;

template<typename T> struct vector
{
    T x, y, z;
};

vector<float> prevAverage;
vector<float> currAverage;
vector<float> variance;
unsigned long timer;
unsigned long averagingTime = 10;
int numSamples = 0;
int numTimeSlots = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  
  prevAverage.x = 0.0;
  prevAverage.y = 0.0;
  prevAverage.z = 0.0;
  currAverage.x = 0.0;
  currAverage.y = 0.0;
  currAverage.z = 0.0;
  variance.x = 0.0;
  variance.y = 0.0;
  variance.z = 0.0;


  Serial.println("Calculating allan variance for gyro.");
  Serial.println("Will calculate variance for X, Y, and Z vectors starting at 1 sec averaging time.");
  Serial.println("The variance is expected to decrease as averaging time increases, then eventually will start to increase again.");
  Serial.println("The averaging time for which the variance is a minimum is the time which is best to calculate the gyro's bias.");
  Serial.println("This sketch will run forever, increasing the averaging time by 1 sec.");
  Serial.println("Need to determine manually which is the best averaging time.");
  timer = millis();  
}

void loop() {
  gyro.read();

  unsigned long currentTime = millis();

  //this time slot is done averaging values
  if(currentTime - timer >= averagingTime)
  {
      //Calculate variance for this timeslot and add it to running total.
      if(numTimeSlots != 0)
      {
          variance.x += pow(prevAverage.x - currAverage.x, 2);
          variance.y += pow(prevAverage.y - currAverage.y, 2);
          variance.z += pow(prevAverage.z - currAverage.z, 2);
      }

      //Set previous average to be current average
      prevAverage.x = currAverage.x;
      prevAverage.y = currAverage.y;
      prevAverage.z = currAverage.z;
      
      //Increment num of calculated time slots and set num of samples for current timeslot to be 0. 
      //Set timer to be 0 to start calculating new time slot
      numSamples = 0;
      numTimeSlots++;
      timer = currentTime;

      //We're done calculating for this averaging time. Calculate the Allan variance for this averaging time
      //Then increase averaging time and reset all variables to calcuate the next allan variance
      if(numTimeSlots == desiredNumTimeSlots)
      {
          vector<float> AVAR;
          AVAR.x = sqrt(variance.x / (2 * (numTimeSlots - 1)));
          AVAR.y = sqrt(variance.y / (2 * (numTimeSlots - 1)));
          AVAR.z = sqrt(variance.z / (2 * (numTimeSlots - 1)));

          Serial.println("----------------");
          Serial.print("Allan Variance for averaging time = ");
          Serial.println(averagingTime);
          Serial.print("X: ");
          Serial.println(AVAR.x);
          Serial.print("Y: ");
          Serial.println(AVAR.y);
          Serial.print("Z: ");
          Serial.println(AVAR.z);

          //Reset for a new allan variance with a new averaging time
          numTimeSlots = 0;
          prevAverage.x = 0.0;
          prevAverage.y = 0.0;
          prevAverage.z = 0.0;
          currAverage.x = 0.0;
          currAverage.y = 0.0;
          currAverage.z = 0.0;
          variance.x = 0.0;
          variance.y = 0.0;
          variance.z = 0.0;
          averagingTime = (AllanNumber % 10 != 0) ? (AllanNumber % 10)*pow(10, AllanNumber / 10) * 10 : pow(10, AllanNumber / 10) * 10; //increase averaging time
          AllanNumber++;
      }
  }

  //Keep a running average of current value of the gyro
  currAverage.x = (currAverage.x*numSamples + gyro.g.x) / (numSamples + 1);
  currAverage.y = (currAverage.y*numSamples + gyro.g.y) / (numSamples + 1);
  currAverage.z = (currAverage.z*numSamples + gyro.g.z) / (numSamples + 1);
  numSamples++;
}
