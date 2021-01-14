#define ADC_VOLTAGE_IN  A0
#define ADC_CURRENT_IN  A1
#define ADC_VOLTAGE_OUT A7

unsigned long timeStamp_verbose = 0;

#define acs712VoltsPerAmp 0.066
#define voltageDividerVoltsPerVolt 0.105662768
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
  //Iin_raw = -(vcc * (analogRead(ADC_CURRENT_IN) - 512) / 1023.) / (2.45*0.066) - 0.08;
  Iin_raw = -((analogRead(ADC_CURRENT_IN) - 512)/1024.) * vcc / acs712VoltsPerAmp;
  Iin_filter = k1*Iin_filter + k*Iin_raw;
  min_sync_pwm = 255 * Vout_filter / Vin_filter;
  myConstrain(min_sync_pwm, pwmMin, pwmMax)


  if ((ACSR & 0x20) == zc_event_val)
  {
    bemfPhaseA = 0.3 + 0.7*bemfPhaseA;
    if (!positive && bemfPhaseA > 0)
    {
      ++commutations;
      positive = true;
    }
  }
  else
  {
    bemfPhaseA = -0.3 + 0.7*bemfPhaseA;
    if (positive && bemfPhaseA <= 0)
    {
      ++commutations;
      positive = false;
    }
  }
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
        if (verboseLevel) Serial.println(Iin_filter);
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
            float inputWatt = Vin_filter*Iin_filter;
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
            Serial.print(F(" Ii"));
            Serial.print(Iin_filter,2);
            Serial.print(F(" W"));
            Serial.print(inputWatt,0);
            if (state == RUNNING_CHARGE)
            {
              Serial.print(F(" Ie"));
              Serial.print(proportionalError,2);
            }
            Serial.print(F(" rpm"));
            Serial.println(60.*commutations/(t*48.),0);
          }
          break;
        default:
          break;
      }
      commutations = 0;
      timeStamp_verbose = millis();
      //Serial.println(timeStamp_verbose);
    }
  }
}
