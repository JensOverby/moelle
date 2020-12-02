#define ADC_VOLTAGE_IN  A0
#define ADC_CURRENT_IN  A1
#define ADC_VOLTAGE_OUT A7

// Define the ADC channels used.  ADLAR will be zero.
//#define ADCH_VOLTAGE_IN ((1 << REFS0) | ADC_VOLTAGE_IN)
//#define ADCH_CURRENT_IN ((1 << REFS0) | ADC_CURRENT_IN)
//#define ADCH_VOLTAGE_OUT ((1 << REFS0) | ADC_VOLTAGE_OUT)

//#define IinPin A1
//#define VinPin A0
//#define VoutPin A7

unsigned int timeStamp_verbose_inSeconds=0, timeStamp_voltage_inSeconds=0;



/*volatile int analogValue_voltage_in;
volatile int analogValue_current_in;
volatile int analogValue_voltage_out;
volatile int adcChan;



void AdcIntSetup()
{
  // Set for the first ADC channel.
  ADMUX = ADCH_VOLTAGE_IN;

  // Set ADEN, prescale (128), and enable interrupts.
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);

  // Clear everything.  No free running because ADATE is clear in ADSRA.
  ADCSRB = 0;

  // Enable global interrupts.
  sei();

  // Kick off the first ADC.
  analogValue_voltage_in = 0;
  analogValue_current_in = 0;
  analogValue_voltage_out = 0;

  // Set ADSC to start ADC conversion.
  ADCSRA |= (1 << ADSC);
}

// The ADC interrupt function.
ISR(ADC_vect)
{
  // Get the ADC channel causing the interrupt.
  int adcChanTmp = ADMUX & ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));

  switch (adcChanTmp)
  {
    case ADC_VOLTAGE_IN:
      analogValue_voltage_in = ADCL | (ADCH << 8);
      ADMUX = ADCH_CURRENT_IN;
      // Set ADSC to start the next ADC conversion.
      ADCSRA |= (1 << ADSC);
      break;
    case ADC_CURRENT_IN:
      analogValue_current_in = ADCL | (ADCH << 8);
      ADMUX = ADCH_VOLTAGE_OUT;
      // Set ADSC to start the next ADC conversion.
      ADCSRA |= (1 << ADSC);
      break;
    case ADC_VOLTAGE_OUT:
      analogValue_voltage_out = ADCL | (ADCH << 8);
      ADMUX = ADCH_VOLTAGE_IN;
      break;
  }

  adcChan = adcChanTmp;
}

void sample_interrupt(float k=0.02)
{
  float k1 = 1. - k;
  while (adcChan != ADC_VOLTAGE_OUT);
  adcChan = ADC_VOLTAGE_IN;
  int a_voltage_in = analogValue_voltage_in;
  int a_current_in = analogValue_current_in;
  int a_voltage_out = analogValue_voltage_out;
  // Set ADSC to start the next ADC conversion.
  ADCSRA |= (1 << ADSC);
  
  Vin_raw = vcc*10.*(float(a_voltage_in)/1023.);
  Vin_filter = k1*Vin_filter + k*Vin_raw;
  Vout_raw = vcc*10.*(float(analogValue_voltage_out)/1023.);
#ifdef DEBUG
  Vout_filter_dbg = k1*Vout_filter_dbg + k*Vout_raw;
  Vout_raw = 12.3;
#endif
  Vout_filter = k1*Vout_filter + k*Vout_raw;
  Iin_raw = -(vcc * float(analogValue_current_in - 512) / 1024.) / 0.066; // - 0.04;
  Iin_filter = k1*Iin_filter + k*Iin_raw;
}*/

//  Vin_raw = 5./vcc * 47.9910483*(analogRead(ADC_VOLTAGE_IN)/1023.);
//  Vout_raw = 5./vcc * 47.9910483*(analogRead(ADC_VOLTAGE_OUT)/1023.);

void sample(float k=0.02)
{
  float k1 = 1. - k;
  Vin_raw = vcc*9.335*(analogRead(ADC_VOLTAGE_IN)/1023.);
  Vin_filter = k1*Vin_filter + k*Vin_raw;
  Vout_raw = vcc*9.335*(analogRead(ADC_VOLTAGE_OUT)/1023.);
#ifdef DEBUG
  Vout_filter_dbg = k1*Vout_filter_dbg + k*Vout_raw;
  Vout_raw = 12.3;
#endif
  Vout_filter = k1*Vout_filter + k*Vout_raw;

  //tmpI = (1.-0.002)*tmpI + 0.002*analogRead(ADC_CURRENT_IN);
  //Iin_raw = 5./vcc * (-0.047605634*analogRead(ADC_CURRENT_IN) + 24.183661972) + 0.04;// - 0.02;
  //Iin_raw = -(vcc * (analogRead(ADC_CURRENT_IN) - 512) / 1024.) / 0.066 - 0.36;
  Iin_raw = -(vcc * (analogRead(ADC_CURRENT_IN) - 512) / 1023.) / (2.45*0.066) - 0.08;
  Iin_filter = k1*Iin_filter + k*Iin_raw;
  min_sync_pwm = 255 * Vout_filter / Vin_filter;
  myConstrain(min_sync_pwm, pwmMin, pwmMax)
}

void breakNow(int voltageBegin=0, int voltageEnd=0, int duty=255, int rampedDuty=255, bool dump=false)
{
  digitalWrite(enableDriverPin, false);
  unsigned long lastTimeDuty = 0;
  float ramped_duty = 0;

  if (dump)
    Serial.println(F("Dumping V -> amps/volt (dI/dV) curve (voltage,A/V):"));
  
  unsigned long timeBegin = millis();
  while (true)
  {
    sample();
    if (millis() > timeBegin + 20000)
    {
      Serial.println(F("Timeout"));
      break;
    }

    if (Vin_filter > voltageBegin)
      break;
  }

  timeBegin = millis();
  while (true)
  {
    sample();

    unsigned long currentTime = millis();
    if (currentTime > timeBegin + 20000)
    {
      Serial.println(F("Timeout"));
      break;
    }

    if((Vin_filter < voltageEnd) || (Vout_filter < VoutMin) || (Vin_filter < (Vout_filter+0.4)))
      break;

    if (currentTime > (lastTimeDuty + 50))
    {
      if (ramped_duty < duty)
        ramped_duty++;
      analogWrite(dumploadPin, ramped_duty);

      lastTimeDuty = currentTime;

      if (dump)
      {
        Serial.print(Vin_filter);
        Serial.print(F(","));
        Serial.println(Iin_filter);
      }
    }
  }
  analogWrite(dumploadPin, 0);
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

void dump(unsigned int nowInSeconds=0)
{
  if (verboseLevel > 0)
  {
    if (nowInSeconds==0 || nowInSeconds > timeStamp_verbose_inSeconds)
    {
      float inputWatt = Vin_filter*Iin_filter;
      Serial.print(F("duty="));
      Serial.print(duty_cycle);
      Serial.print(F(", Vin="));
      Serial.print(Vin_filter);
      Serial.print(F("V, Iin_filter="));
      Serial.print(Iin_filter);
      Serial.print(F("A, Vout="));
    #ifdef DEBUG
      float Iout = efficiency*inputWatt/Vout_filter_dbg;
      Serial.print(Vout_filter_dbg);
    #else
      float Iout = efficiency*inputWatt/Vout_filter;
      Serial.print(Vout_filter);
    #endif  
      Serial.print(F("V, Iout="));
      Serial.print(Iout);
      Serial.print(F("A, watt="));
      Serial.print(inputWatt);
      Serial.print(F(",spwm="));
      Serial.println(min_sync_pwm);
      timeStamp_verbose_inSeconds = nowInSeconds + 1;
    }
  }
}

void dumpCharging(unsigned int nowInSeconds)
{
  if (nowInSeconds > timeStamp_voltage_inSeconds)
  {
    switch (verboseLevel)
    {
      if (Serial.availableForWrite() >= 64)
      {
        case 1:
          Serial.println(int(duty_cycle));
          break;
        case 2:
          {
            float inputWatt = Vin_filter*Iin_filter;
            Serial.print(F("du"));
            Serial.print(int(duty_cycle));
            Serial.print(F(",Vi"));
            Serial.print(Vin_filter);
            Serial.print(F(",Vo"));
            Serial.print(Vout_filter);
            Serial.print(F(",I"));
            Serial.print(Iin_filter);
            Serial.print(F(",W"));
            Serial.print(inputWatt);
            Serial.print(F(",Ie"));
            Serial.print(proportionalError);
            Serial.print(F(",ofs"));
            Serial.println(sync_pwm_positive_offset);
          }
          break;
        default:
          break;
      }
    }
    timeStamp_voltage_inSeconds = nowInSeconds;
  }
}
