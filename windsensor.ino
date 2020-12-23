#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;

float pressure_mean=1005, pressure_stddev_mean=0;

//unsigned int lastGustTimeInSeconds = 0;
//byte gustCounter = 0;
bool initSuccessful = false;

bool initWindSensor()
{
  // Initialize pressure sensor
  if (initSuccessful = pressure.begin())
    Serial.println(F("BMP180 init success"));
  else
  {
    Serial.println(F("BMP180 init fail (disconnected?)\n\n"));
  }
  return initSuccessful;
}

bool getWindSpeedOK(bool verbose, float* std)
{
  if (!initSuccessful)
    return false;
  float std_tmp;
  if (std==NULL)
    std = &std_tmp;
  char status = pressure.startPressure(3);
  *std = 0;
  if (status != 0)
  {
    waitMS(status);
    double T = 15, P;
    status = pressure.getPressure(P,T);
    if (status == 0)
    {
      if (verboseLevel) Serial.println(F("error retrieving pressure measurement"));
      return false;
    }

    pressure_stddev_mean = 0.1*pow(P-pressure_mean,2) + 0.9*pressure_stddev_mean;
    pressure_mean = 0.01*P + 0.99*pressure_mean;
    *std = sqrt(pressure_stddev_mean);
    if (verbose) Serial.print(F(" P="));
    if (verbose) Serial.print(P);
    if (verbose) Serial.print(F(" mean="));
    if (verbose) Serial.print(pressure_mean);
    if (verbose) Serial.print(F(" std="));
    if (verbose) Serial.print(*std);
    if (verbose) Serial.print(F(" thres="));
    if (verbose) Serial.print(pressureThreshold);
    if (verbose) Serial.print(F(" gusts="));
    if (verbose) Serial.println(gustCounter);
  }
  else if (verboseLevel)
    Serial.println(F("error starting pressure measurement"));

  unsigned int nowInSeconds = millis() / (1000*TT);
  if (*std > pressureThreshold)
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

  return false;
}

void getPressure(double& P, double& T)
{
  char status;
  double p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    waitMS(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    //Serial.print("temp = ");
    //Serial.println(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        waitMS(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
        T = 25.0;
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return;
        }
        else if (verboseLevel) Serial.println(F("error retrieving pressure measurement\n"));
      }
      else if (verboseLevel) Serial.println(F("error starting pressure measurement\n"));
    }
    else if (verboseLevel) Serial.println(F("error retrieving temperature measurement\n"));
  }
  else if (verboseLevel) Serial.println(F("error starting temperature measurement\n"));
}

void windSpeedClear()
{
  pressure_stddev_mean = 0.0;
}

void windSensorFeedback(bool success)
{
  if (success)
    pressureThreshold -= 0.01;
  else
    pressureThreshold += 0.01;
}
