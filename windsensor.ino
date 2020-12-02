#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;

double pressureThreshold = 0.01;
double pressure_mean, pressure_stddev_mean;

unsigned long windSensorTime = 0;

bool initWindSensor()
{
  // Initialize pressure sensor
  if (pressure.begin())
    Serial.println(F("BMP180 init success"));
  else
  {
    Serial.println(F("BMP180 init fail (disconnected?)\n\n"));
    return false;
    //while(1); // Pause forever.
  }
  return true;
}

void getPressure(double& P, double& T)
{
  return;
  
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

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
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
        delay(status);

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
        else Serial.println(F("error retrieving pressure measurement\n"));
      }
      else Serial.println(F("error starting pressure measurement\n"));
    }
    else Serial.println(F("error retrieving temperature measurement\n"));
  }
  else Serial.println(F("error starting temperature measurement\n"));
}

void windSpeedClear()
{
  windSensorTime = millis();
  pressure_stddev_mean = 0.0;
  double T;
  getPressure(pressure_mean, T);
}

bool getWindSpeedOK()
{
  double pressure, temp;
  getPressure(pressure, temp);
  pressure_mean += 0.01 * (pressure - pressure_mean);
  float pressure_stddev = fabs(pressure - pressure_mean);
  pressure_stddev_mean = 0.1 * (pressure_stddev - pressure_stddev_mean);
  if (fabs(pressure_stddev_mean) > 0.005)
  {
    Serial.print(F("std dev: "));
    Serial.print(pressure_stddev, 4);
    Serial.print(F(", mean: "));
    Serial.print(pressure_stddev_mean, 4);
    Serial.print(F(", pressure: "));
    Serial.println(pressure, 4);
    Serial.print(F(", temp: "));
    Serial.println(temp, 4);
  }

  if (pressure_stddev_mean > pressureThreshold)
  {
    unsigned long now = millis();
    if ((now - windSensorTime) < 5000)
      return true;
    windSensorTime = now;
  }
  
  return false;
}

void windSensorFeedback(bool success)
{
  if (success)
    pressureThreshold -= 0.002;
  else
    pressureThreshold += 0.002;
}
