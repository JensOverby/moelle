#include<Wire.h>
#include <math.h>

/*const int MPU=0x68;

float AcXmean = 0;
float AcYmean = 0;
float AcZmean = 0;

unsigned int lastGustTimeInSeconds = 0;
byte gustCounter = 0;*/

unsigned int spinUpPeriod = 1;
unsigned long lastSpinTimeInSeconds = 0;
unsigned int errorCount = 0;

bool initWindSensor2()
{
  spinUpPeriod = 1;
  lastSpinTimeInSeconds = millis()/(1000*TT);
  return true;

  /*Serial.print("1");
  waitMS(200);
  Wire.begin();
  Serial.print("2");
  waitMS(200);
  Wire.beginTransmission(MPU);
  Serial.print("3");
  waitMS(200);
  Wire.write(0x6B);
  Serial.print("4");
  waitMS(200);
  Wire.write(0);
  Serial.print("5");
  waitMS(200);
  Wire.endTransmission(true);
  Serial.println(" ok");
  return true;*/
}

bool getWindSpeedOK2(bool verbose)
{
  unsigned long nowInSeconds = millis() / (1000*TT);
  unsigned long diff = (nowInSeconds - lastSpinTimeInSeconds) / 60;

  if (diff > spinUpPeriod || (diff > 1 && errorCount > 0))
  {
    lastSpinTimeInSeconds = nowInSeconds;
    return true;
  }
  return false;
  
  /*Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  
  //read accel data
  int AcX=(Wire.read()<<8|Wire.read());
  int AcY=(Wire.read()<<8|Wire.read());
  int AcZ=(Wire.read()<<8|Wire.read());

  AcXmean = 0.9*AcX + 0.1*AcXmean;
  AcYmean = 0.9*AcY + 0.1*AcYmean;
  AcZmean = 0.9*AcZ + 0.1*AcZmean;

  float x = AcX - AcXmean;
  float y = AcY - AcYmean;
  float z = AcZ - AcZmean;
  float acc = sqrt(x*x + y*y + z*z)/333.;

  unsigned int nowInSeconds = millis() / (1000*TT);
  if (acc > pressureThreshold)
  {
    if (nowInSeconds > (lastGustTimeInSeconds + gust_spacing_min))
    {
      if (nowInSeconds < (lastGustTimeInSeconds + gust_spacing_max))
      {
        gustCounter++;
        if (gustCounter >= required_number_of_gusts)
        {
          gustCounter = 0;
          return true;
        }
      }
      else
        gustCounter=1;

      lastGustTimeInSeconds = nowInSeconds;
    }
  }
  else if (nowInSeconds > (lastGustTimeInSeconds + gust_spacing_max))
    gustCounter = 0;

  if (verbose) Serial.print(F("acc = "));
  if (verbose) Serial.print(acc);
  if (verbose) Serial.print(F(" thres="));
  if (verbose) Serial.print(pressureThreshold);
  if (verbose) Serial.print(F(" gusts="));
  if (verbose) Serial.println(gustCounter);

  return false;*/
}

void windSpeedClear2()
{
}

void windSensorFeedback2(WindFeedbackType value)
{
  switch (value)
  {
    case SUCCESS:
      errorCount = 0;
      spinUpPeriod = 1;
      Serial.println(F("windsensor: No update"));
      break;
    case COM_ERROR:
      errorCount++;
      if (errorCount < 2)
        break;
    case FAIL:
      errorCount = 0;
      spinUpPeriod *= 2;
      if (spinUpPeriod > 128)
        spinUpPeriod = 128;
      Serial.println(F("windsensor: added lazyness"));
    break;
    default:
    break;
  }

  Serial.print(F("spinUpPeriod="));
  Serial.println(spinUpPeriod);
  
  /*if (success)
  {
    if (pressureThreshold > 0.08)
      pressureThreshold -= 0.01;
  }
  else
    pressureThreshold += 0.01;

  lastSpinTimeInSeconds = millis()/(1000*TT);*/
}
