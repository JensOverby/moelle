#define ADC_VOLTAGE_IN  A0
#define ADC_CURRENT_OUT  A1
#define ADC_VOLTAGE_OUT A7

unsigned long timeStamp_verbose = 0;
unsigned long rpm_time = 0;

#define acs712VoltsPerAmp 0.066
#define voltageDividerVoltsPerVolt 0.089637248
//(5.5/47.)

bool positive = true;
void sample(float k)
{
  float k1 = 1. - k;
  //Vin_raw = vcc*9.335*(analogRead(ADC_VOLTAGE_IN)/1023.);
  Vin_raw = (analogRead(ADC_VOLTAGE_IN)/1024.) * vcc / voltageDividerVoltsPerVolt;
  Vin_filter = k1*Vin_filter + k*Vin_raw;
  //Vout_raw = vcc*9.335*(analogRead(ADC_VOLTAGE_OUT)/1023.);
  Vout_raw = (analogRead(ADC_VOLTAGE_OUT)/1024.) * vcc / voltageDividerVoltsPerVolt;
#ifdef DEBUG
  Vout_filter_dbg = k1*Vout_filter_dbg + k*Vout_raw;
  Vout_raw = 12.3;
#endif
  Vout_filter = k1*Vout_filter + k*Vout_raw;
  //Iout_raw = -(vcc * (analogRead(ADC_CURRENT_IN) - 512) / 1023.) / (2.45*0.066) - 0.08;
  Iout_raw = -((analogRead(ADC_CURRENT_OUT) - 508)/1024.) * 5. / acs712VoltsPerAmp;// - 0.02;
  Iout_filter = k1*Iout_filter + k*Iout_raw;
  min_sync_pwm = 255 * Vout_filter / Vin_filter;
  myConstrain(min_sync_pwm, pwmMin, pwmMax)


  if ((ACSR & 0x20) == zc_event_val)
  {
    bemfPhaseA = 0.3 + 0.7*bemfPhaseA;
    if (!positive && bemfPhaseA > 0)
    {
      //++commutations;
      unsigned long t = millis();
      unsigned long dt = t-rpm_time;
      rpm_time = t;
      if (dt <= 0)
        dt = 1;
      float rps = 32000./(dt*48.);
      rpm_filter = k1*rpm_filter + k*rps*60;
      positive = true;
    }
  }
  else
  {
    bemfPhaseA = -0.3 + 0.7*bemfPhaseA;
    if (positive && bemfPhaseA <= 0)
    {
      //++commutations;
      unsigned long t = millis();
      int dt = t-rpm_time;
      rpm_time = t;
      if (dt <= 0)
        dt = 1;
      float rps = 32000./(dt*48.);
      rpm_filter = k1*rpm_filter + k*rps*60;
      positive = false;
    }
  }
}

#define RPM_TO_WINDSPEED_PROPORTIONAL_FACTOR 0.01720396 // = circumference / (tip_speed_ratio*60), where tsr=7 and circum=2*pi*r
#define POWER_FACTOR 0.7935   // = 0.5*blade_efficiency*Area, where blade_efficiency Cp=0.38197 and Area=pi*r^2
// Radius r = 1.15 meter

// Cp=0.38197 is Hugh Piggotts value, but is 0.3 in http://www.windandwet.com/windturbine/power_calc/index.php
// and 0.4 in http://www.ijsrp.org/research_paper_feb2012/ijsrp-feb-2012-06.pdf
      
float getExpectedPower(float rpm)
{
  float windspeed = rpm * RPM_TO_WINDSPEED_PROPORTIONAL_FACTOR; // Unit is meter/sec
  float power = POWER_FACTOR * pow(windspeed, 3);
  return power;
}


float interpolate(float x0, float y0, float x1, float y1, float x) { return y0 + (x-x0)/(x1-x0) * (y1-y0); }
float getInterpolatedY(float x, float* pData, int* data_ptr)
{
  if (x > *pData)
    return *(pData+1);
  int data_sz = sizeof(pData)/4;
  if (x <= *(pData+data_sz-2))
    return *(pData+data_sz-1);
  while (*(pData+(*data_ptr)*2+2) > x)
    (*data_ptr)++;
  while (*(pData+(*data_ptr)*2) <= x)
    (*data_ptr)--;
  float y = interpolate(*(pData+(*data_ptr)*2),*(pData+(*data_ptr)*2+1),*(pData+(*data_ptr)*2+2),*(pData+(*data_ptr)*2+3),x);
  return y;
}

void breakNow(int voltageBegin, int voltageEnd, bool dump)
{
  digitalWrite(enableDriverPin, false);
  unsigned long lastTimeDuty = 0;
  float ramped_duty = 0;

  if (dump)
    if (verboseLevel) Serial.println(F("Dumping V -> amps/volt (dI/dV) curve (voltage,A/V):"));
  
  unsigned long now = millis(), nowInSeconds = now/(1000*TT);
  unsigned long timeOut = nowInSeconds + 20;
  while (true)
  {
    sample(0.1);
    now = millis(); nowInSeconds = now/(1000*TT);
    if (nowInSeconds > timeOut)
    {
      if (verboseLevel) Serial.println(F("Timeout"));
      break;
    }

    if (Vin_filter > voltageBegin)
      break;
  }

  timeOut = nowInSeconds + 20;
  digitalWrite(dumploadPin, true);
  while (true)
  {
    sample(0.1);
    now = millis(); nowInSeconds = now/(1000*TT);
    if (nowInSeconds > timeOut)
    {
      if (verboseLevel) Serial.println(F("Timeout"));
      break;
    }

    if((Vin_filter < voltageEnd) || (Vout_filter < VoutMin) || (Vin_filter < (Vout_filter+0.4)))
      break;

    if (now > (lastTimeDuty + 50*TT))
    {
      lastTimeDuty = now;

      if (dump)
      {
        if (verboseLevel) Serial.print(Vin_filter);
        if (verboseLevel) Serial.print(F(","));
        if (verboseLevel) Serial.println(Iout_filter);
      }
    }
  }
  digitalWrite(dumploadPin, false);
}

float readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  unsigned int old = ADMUX;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  waitMS(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  ADMUX = old;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result/1000.;
}

void dump()
{
  float t = (millis() - timeStamp_verbose) / (1000.*TT);
  if (t > 2)
  {
    if (Serial.availableForWrite() >= 48)
    {
      switch (verboseLevel)
      {
        case 1:
          Serial.println(int(duty_cycle));
          break;
        case 2:
          {
            float outputWatt = Vout_filter*Iout_filter;
            Serial.print(F("du"));
            Serial.print(int(duty_cycle));
            Serial.print(F(" Vi"));
            Serial.print(Vin_filter,1);
            Serial.print(F(" Vo"));
      #ifdef DEBUG
            Serial.print(Vout_filter_dbg,1);
      #else
            Serial.print(Vout_filter,1);
      #endif
            Serial.print(F(" Io"));
            Serial.print(Iout_filter,2);
            Serial.print(F(" W"));
            Serial.print(outputWatt,0);
            if (state == RUNNING_CHARGE)
            {
              Serial.print(F(" Ie"));
              Serial.print(proportionalError,2);
            }
            Serial.print(F(" rpm"));
            Serial.println(rpm_filter,0);
          }
          break;
        default:
          break;
      }
      //commutations = 0;
      timeStamp_verbose = millis();
      //Serial.println(timeStamp_verbose);
    }
  }
}
