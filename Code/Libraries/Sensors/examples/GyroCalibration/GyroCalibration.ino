#include <EEPROMAnything.h>
#include <Wire.h>
#include <L3G.h>
#include <eeprom.h>

L3G gyro;

template<typename T> struct vector
{
    T x, y, z;
};
const float ROTATION_ANGLE = 360.0f;
const float READING_TO_DPS = 0.00875f;
float scaleFactor = 0.0f;
float averageBiasZ = 0.0f;
float sigmaZ = 0.0f;
float angleZ = 0.0f;
float maxAngleZ = 0.0f;
int numSamples = 0;
float M2 = 0.0f;
unsigned long currentTime;
unsigned long timerBias = 0;
unsigned long timerNoise = 0;
unsigned long timerScale = 0;
unsigned long timerSample = 0;
unsigned long averagingTime = 5000; //Optimal time based on allan variance is 5 sec
unsigned long sampleTime = 10;

bool printResults = true;
bool measureScaleFactor = true;
bool hasRotated = false;
bool hasPrinted = false;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    if (!gyro.init())
    {
        Serial.println("Failed to autodetect gyro type!");
        while(1);
    }
    gyro.enableDefault();

    averageBiasZ = 0.0f;
    sigmaZ = 0.0f;
    angleZ = 0.0f;
    maxAngleZ = 0.0f;
    timerBias = millis();
}

void loop()
{
    gyro.read();

    unsigned long currentTime = millis();
    //Check if we have calculated bias for the desired amount of time.
    if(currentTime - timerBias < averagingTime)
    {
        //Keep a running average of current value of the gyro
        numSamples++;
        float delta = (float) gyro.g.z - averageBiasZ;
        averageBiasZ += delta/numSamples;
        M2 += delta*((float) gyro.g.z - averageBiasZ);
    }
    if(currentTime - timerNoise >= averagingTime && printResults)
    {
        //calculate sigmaZ, the standard deviation of the Z values
        float variance = M2 / (numSamples - 1);
        sigmaZ = sqrt(variance);

        //Print results
        Serial.println("Average Bias:");
        Serial.print("Z= ");
        Serial.println(averageBiasZ);

        Serial.println("Sigma:");
        Serial.print("Z= ");
        Serial.println(sigmaZ);
        printResults = false;
    }

    //Measure Scale Factor for Z
    if(!printResults && measureScaleFactor)
    {
        if(!hasPrinted)
        {
            Serial.print("Rotate clockwise to ");
            Serial.print(ROTATION_ANGLE);
            Serial.println(" and back to 0");
            Serial.println("3...");
            delay(300);
            Serial.println("2...");
            delay(300);
            Serial.println("1...");
            delay(300);
            Serial.println("GO");
            hasPrinted = true;
            currentTime = millis();
            timerScale = currentTime;
        }
        else
        {
            sampleTime = currentTime - timerSample;
            timerSample = currentTime;

            //current rate of rotation
            float rateZ = ((float) gyro.g.z - averageBiasZ) * READING_TO_DPS;

            //find angle
            angleZ += (rateZ * sampleTime / 1000.0f); //divide by 1000(convert to sec)

            Serial.print("Angle = "); Serial.print(angleZ); Serial.print("\tRate = "); Serial.println(rateZ);

            //Find max angle
            if(angleZ > maxAngleZ) maxAngleZ = angleZ;

            //After 1 sec has passed, check if we've stopped at degree 90 or back where we started
            if(currentTime - timerScale >= 3000UL)
            {
                //We've stopped turning; we must be at degree 90 or back where we started.
                if(abs(rateZ) < 0.1f)
                {
                    //If we havent rotated to 90 yet
                    if(!hasRotated)
                    {
                        Serial.println("rotate back to starting position.");
                        timerScale = currentTime;
                        hasRotated = true;
                    }
                    else //we're back where we started. Measure scale factor. Save bias and scale factor to eeprom, then output current readings.
                    {
                        //Scale factor is the actual amount we rotated the robot divided by the gyro's measured rotation angle
                        //Also includes the sensitivity value
                        scaleFactor = ROTATION_ANGLE / maxAngleZ * READING_TO_DPS; 
                        Serial.println(maxAngleZ);
                        Serial.println(scaleFactor);
                        measureScaleFactor = false;
                        
                        //Write the averageZ bias to the eeprom
                        int nextAddress = EEPROM_writeAnything(0, averageBiasZ);
                        //Write the noiseZ to the eeprom
                        nextAddress += EEPROM_writeAnything(nextAddress, sigmaZ);
                        //Write the scale factor to eeprom
                        EEPROM_writeAnything(nextAddress, scaleFactor);

                        //Print out the values put into the eeprom for verification:
                        float storedBias;
                        float storedSigma;
                        float storedScaleFactor;
                        nextAddress = EEPROM_readAnything(0, storedBias);
                        nextAddress += EEPROM_readAnything(nextAddress, storedSigma);
                        nextAddress += EEPROM_readAnything(nextAddress, storedScaleFactor);

                        Serial.print("BiasZ = ");
                        Serial.print(averageBiasZ);
                        Serial.print("\tStored = ");
                        Serial.println(storedBias);

                        Serial.print("sigmaZ = ");
                        Serial.print(sigmaZ);
                        Serial.print("\tStored = ");
                        Serial.println(storedSigma);

                        Serial.print("scaleFactorZ = ");
                        Serial.print(scaleFactor);
                        Serial.print("\tStored = ");
                        Serial.println(storedScaleFactor);
                    }
                }
            }
        }        
    }

  //Done measuring scale factor. Measure current angle & rate
  if(!measureScaleFactor)
  {  
      sampleTime = currentTime - timerSample;
      timerSample = currentTime;

      //current rate of rotation
      float rateZ = ((float) gyro.g.z - averageBiasZ);
      if(abs(rateZ) < 3 * sigmaZ)
      {
          rateZ = 0.0f;
      }

      //find angle
      angleZ += (rateZ * sampleTime / 1000.0f) * scaleFactor; //divide by 1000(convert to sec)

      // Keep our angle between 0-359 degrees
      if(angleZ < 0) angleZ += 360;
      else if(angleZ >= 360) angleZ -= 360;

      Serial.print("Angle = "); Serial.print(angleZ); Serial.print("\tRate = "); Serial.println(rateZ * scaleFactor);
  }
}
