#include <EEPROM.h>

enum WorkSpcRole {CMD_IDLE=0,
                  CMD_HELP=1,
                  CMD_VERBOSE=2,
                  CMD_DUTY=3,
                  CMD_DUMPLOAD=4,
                  CMD_ACKNOWLEDGE=5,
                  CMD_SPINUP=6,
                  CMD_ENABLE_WIND_SENSOR=7,
                  CMD_SAMPLE=8,
                  CMD_DIAGNOSTICS=9,
                  CMD_SIMRUN=11,
                  CMD_PRINTSETTINGS=12,
                  CMD_SET_SETTING=13,
                  CMD_STORE_SETTINGS=14,
                  //CMD_LOAD_FACTORY_SETTINGS=11,
                  //CMD_POWERCURVE=12,
                  //CMD_SET_POWER=13,
                  //CMD_STORE_POWERCURVE=14,
                  CMD_WIND_PARAMS=15,
                  CMD_MIN_RUNTIME=16,
                  CMD_SAMPLE_STARTUP_WIND=17,
                  CMD_COMMUTATE=18,
                  CMD_STOP=19};
                  
char commandBuffer[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static unsigned int bytesReceived = 0;
static unsigned int bytesExpected = 0;
static unsigned int hashValue = 0;

void printHelp()
{
  Serial.println(F("Commands:"));
  Serial.println(F("-----------------"));
  Serial.println(F("help | prints this message"));
  Serial.println(F("ack | toggle self startup"));
  Serial.println(F("spinup | spinup turbine now"));
  Serial.println(F("initstartwind | sample startup wind"));
  Serial.println(F("windsense <1/0> | wind sensor on/off"));
  Serial.println(F("sample | sample Vin,Iout,Vout"));
  Serial.println(F("verbose <1/0> | verbose on/off"));
  Serial.println(F("duty <value> | set duty (0-255)"));
  Serial.println(F("dumpload <V-begin> <V-end> | use dump resistor (optional params)"));
  Serial.println(F("simrun <1/0> | simulate turbine run"));
  Serial.println(F("settings | print settings"));
  Serial.println(F("setsetting <id> <val> | set setting by id,value"));
  Serial.println(F("storesettings | store settings"));
  Serial.println(F("sensorparams <threshold> <gustminsecs> <gustmaxsecs> <gustcount> | set wind sensor parameters"));
  Serial.println(F("minruntime <sec> | set minimum ok run time"));
  Serial.println(F("commutate <cnt> | commutate motor forward"));
  Serial.println(F("poweroffset <value> | spinup pwm offset"));
  Serial.println(F("stop <offset> | break to offset voltage above Vout. Default 0"));
  Serial.println();
}

int getCommand()
{
  // If something arrived at VCP
  uint8_t c;
  while (Serial.available() > 0)
  {
    c = Serial.read();
    if (bytesReceived == 0)
    {
      switch(c)
      {
      case CMD_IDLE:
        bytesExpected = 1;
        break;
      default:
        if (c == '\n')
        {
          if (hashValue != 0)
          {
            Serial.print(F("invalid #"));
            Serial.println(hashValue);
          }
          c = CMD_IDLE;
          verboseLevel = verboseLevel_old;
          hashValue = 0;
          bytesExpected = 1;
          break;
          //continue;
        }
        hashValue += c;
        hashValue += (hashValue << 10);
        hashValue ^= (hashValue >> 6);

        switch (hashValue)
        {
        case 18536: // help
          bytesExpected = -1;
          c = CMD_HELP;
          break;
        case 38105: // verbose
          bytesExpected = -1;
          c = CMD_VERBOSE;
          break;
        case 32204: // duty
          bytesExpected = -1;
          c = CMD_DUTY;
          break;
        case 41053: // dumpload
          bytesExpected = -1;
          c = CMD_DUMPLOAD;
          break;
        case 56699:
          bytesExpected = -1;
          c = CMD_ACKNOWLEDGE;
          break;
        case 7077:
          bytesExpected = -1;
          c = CMD_SPINUP;
          break;
        case 861:
          bytesExpected = -1;
          c = CMD_ENABLE_WIND_SENSOR;
          break;
        case 36609:
          bytesExpected = -1;
          c = CMD_SAMPLE;
          break;
        case 31168:
          bytesExpected = -1;
          c = CMD_DIAGNOSTICS;
          break;
        case 58025:
          bytesExpected = -1;
          c = CMD_SIMRUN;
          break;
        case 23878:
          bytesExpected = -1;
          c = CMD_PRINTSETTINGS;
          break;
        case 92:
          bytesExpected = -1;
          c = CMD_SET_SETTING;
          break;
        case 39615:
          bytesExpected = -1;
          c = CMD_STORE_SETTINGS;
          break;
        /*case 54619:
          bytesExpected = -1;
          c = CMD_LOAD_FACTORY_SETTINGS;
          break;
        case 42261:
          bytesExpected = -1;
          c = CMD_POWERCURVE;
          break;
        case 56572:
          bytesExpected = -1;
          c = CMD_SET_POWER;
          break;
        case 55971:
          bytesExpected = -1;
          c = CMD_STORE_POWERCURVE;
          break;*/
        case 51939:
          bytesExpected = -1;
          c = CMD_WIND_PARAMS;
          break;
        case 7018:
          bytesExpected = -1;
          c = CMD_MIN_RUNTIME;
          break;
        case 17904:
          bytesExpected = -1;
          c = CMD_SAMPLE_STARTUP_WIND;
          break;
        case 44599:
          bytesExpected = -1;
          c = CMD_COMMUTATE;
          break;
        case 1561:
          bytesExpected = -1;
          c = CMD_STOP;
          break;
        default:
          break;
        }

        if (bytesExpected == -1)
          break;

        continue;
      }
    }

    if (hashValue != 0 && c == '\n')
    {
      commandBuffer[bytesReceived] = 0;
      hashValue = 0;
      bytesExpected = 0;
      verboseLevel = verboseLevel_old;
    }
    else
      commandBuffer[bytesReceived] = c;

    bytesReceived++;

    if (bytesReceived >= bytesExpected)
    {
      bytesReceived = 0;
      return 1;
    }
  }

  return 0;
}

/*PROGMEM const float factory_powercurve[] =  {26, 17.29,
                                             25, 17.21,
                                             24, 16.48,
                                             23, 14.77,
                                             22, 12.02,
                                             21, 8.75,
                                             20, 5.77,
                                             19, 3.6,
                                             18, 2.25,
                                             17, 1.5,
                                             16, 1.11,
                                             15, 0.92,
                                             14, 0.85};*/

void eepromReadFloat16(float* pMem, int len, int address)
{
  byte value[2];
  unsigned int* iVal = (unsigned int*)value;
  
  for (int i=0; i<len; ++i)
  {
    value[0] = EEPROM.read(i*2 + address);
    value[1] = EEPROM.read(i*2+1 + address);
    pMem[i] = *iVal / 100.; // float from 0 to 655 with 2 decimal places

    /*Serial.print(value[0]);
    Serial.print(",");
    Serial.print(value[1]);
    Serial.print(",");
    Serial.print(*iVal);
    Serial.print(",");
    if (i%2 == 0)
    {
      Serial.print(pMem[i]);
      Serial.print(F(","));
    }
    else
      Serial.println(pMem[i]);*/
  }
}

void eepromUpdateFloat16(float* pMem, int len, int address=0)
{
  unsigned int iVal;
  byte* value = (byte*)&iVal;
  
  for (int i=0; i<len; ++i)
  {
    iVal = int(pMem[i]);
    float dec = pMem[i] - iVal;
    iVal = iVal*100 + round(dec*100);// + 1;
    EEPROM.update(i*2 + address, value[0]);
    EEPROM.update(i*2+1 + address, value[1]);

    /*if (i%2 == 0)
    {
      Serial.print(pMem[i]);
      Serial.print(F(","));
    }
    else
      Serial.println(pMem[i]);
    
    Serial.print(iVal);
    Serial.print(",");
    Serial.print(value[0]);
    Serial.print(",");
    Serial.println(value[1]);*/
  }
}

void sampleDump()
{
  Serial.print(F("V_in="));
  Serial.print(Vin_filter);
  Serial.print(F(" I_out="));
  Serial.print(Iout_filter);
  Serial.print(F(" V_out="));
  Serial.println(Vout_filter);
}

void sampleAndDump(int dumps, int periodMS)
{
  unsigned long nextDump = 0;
  for (int dumpCount=0; dumpCount<dumps;)
  {
    sample();
    if (millis() > nextDump)
    {
      sampleDump();
      dumpCount++;
      nextDump = millis() + periodMS*TT;
    }
  }
}

/*void checkPhase(char* nameStr, int P_IN, int P_SD, bool use_adc=false)
{
  Serial.print(F("Checking phase "));
  Serial.println(nameStr);
  int toggle = 0;
  analogWrite(P_IN, 15);
  digitalWrite(P_SD, HIGH);
  float phaseVal = 0;
  unsigned long nextDump = millis() + 1000L*TT;
  Serial.println(" high low float");
  for (int dumpCount=0; dumpCount<8;)
  {
    if (millis() > nextDump)
    {
      switch (toggle)
      {
        case 0:
          Serial.print(nameStr);
          Serial.print(" 15 240   0 : value = ");
          Serial.println(phaseVal);
          digitalWrite(P_IN, LOW);
          digitalWrite(P_SD, HIGH);
          break;
        case 1:
          Serial.print(nameStr);
          Serial.print("  0 255   0 : value = ");
          Serial.println(phaseVal);
          analogWrite(P_SD, 15);
          digitalWrite(P_IN, LOW);
          break;
        case 2:
          Serial.print(nameStr);
          Serial.print("  0  15   0 : value = ");
          Serial.println(phaseVal);
          analogWrite(P_SD, 15);
          digitalWrite(P_IN, HIGH);
          break;
        default:
          Serial.print(nameStr);
          Serial.print(" 15   0   0 : value = ");
          Serial.println(phaseVal);
          analogWrite(P_IN, 15);
          digitalWrite(P_SD, HIGH);
          toggle = -1;
          break;
      }
      toggle++;
      phaseVal = 0;
      dumpCount++;
      nextDump = millis() + 3000L*TT;
    }

    int val;
    if (use_adc)
    {
      ADCSRA = 135;
      ADCSRB = 0;
      Vin_raw = (analogRead(ADC_VOLTAGE_IN)/1024.) * 5 / voltageDividerVoltsPerVolt;
      phaseVal = 0.01*Vin_raw + 0.99*phaseVal;
    }
    else
    {
      int val = ((ACSR & 0x20) == zc_event_val);
      phaseVal = 0.01*val + 0.99*phaseVal;
    }

  }

  digitalWrite(P_IN, LOW);
  digitalWrite(P_SD, LOW);
}*/

void checkPhase(char* nameStr, int P_IN, int P_SD)
{
  freeWheel();

  Serial.print(F("Checking phase "));
  Serial.println(nameStr);
  Serial.println(F("================"));
  
  Serial.println(F("Buck converter low mosfet ON"));
  pwmLow(15);
  sampleAndDump(9, 500);

  Serial.println(F("Buck converter high mosfet ON"));
  pwmHigh(15);
  sampleAndDump(9, 500);
  
  Serial.println(F("Both"));
  analogWrite(P_IN, 15);
  digitalWrite(P_SD, HIGH);
  sampleAndDump(9, 500);

  Serial.println(F("High"));
  digitalWrite(P_IN, HIGH);
  analogWrite(P_SD, 15);
  sampleAndDump(9, 500);

  Serial.println(F("Low"));
  digitalWrite(P_IN, LOW);
  analogWrite(P_SD, 15);
  sampleAndDump(9, 500);

  Serial.println(F("None"));
  digitalWrite(P_IN, LOW);
  digitalWrite(P_SD, LOW);
  sampleAndDump(9, 500);

  freeWheel();

  Serial.println(F("No buck converter mosfets ON"));
  digitalWrite(enableDriverPin, false);
  sampleAndDump(9, 500);
}

bool execCommand()
{
  switch (commandBuffer[0])
  {
  case CMD_VERBOSE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &verboseLevel);
      verboseLevel_old = verboseLevel;
    }
    break;
  case CMD_DUTY:
    {
      freeWheel();
      
      int pwmVal = -1;
      sscanf((char*)(commandBuffer+1), "%d", &pwmVal);

      if (pwmVal == -1)
        break;
      
      Serial.print(F("pwm duty = "));
      Serial.println(pwmVal);

      for (int i=0; i<100; ++i)
        sample();

      duty_cycle = pwmVal;
      pwm(duty_cycle);

      int counter = 0;
      while (!getCommand())
      {
        sample();

        int minimum_pwm = 255. * Vout_filter / Vin_filter;
        if (minimum_pwm < pwmMin)
          minimum_pwm = pwmMin;
        if (minimum_pwm > pwmMax || pwmVal < minimum_pwm)
        {
          pwmDisable();
          Serial.println(F("aborting, input voltage too low"));
          commandBuffer[0] = CMD_IDLE;
          return false;
        }

        counter++;
        if (counter>100)
        {
          counter=0;
          if (dump())
          {
            Serial.print(F("minpwm="));
            Serial.println(minimum_pwm);
          }
        }
        waitMS(1);
      }
      pwmDisable();
      //digitalWrite(enableDriverPin, false);
    }
    return true;
  case CMD_DUMPLOAD:
    {
      int voltageBegin = -1, voltageEnd = -1;
      sscanf((char*)(commandBuffer+1), "%d %d", &voltageBegin, &voltageEnd);
      if (voltageBegin == -1 || voltageEnd == -1)
        breakNow();
      else
        breakNow(voltageBegin, voltageEnd, true);
    }
    break;
  case CMD_STOP:
    {
      char offs[5]="0";
      sscanf((char*)(commandBuffer+1), "%s", offs);
      float off = atof(offs);
      digitalWrite(enableDriverPin, false);
      //commutations = 0;
      unsigned long stopTimeInMillis = millis()/TT + 10000;
      while (!getCommand())
      {
        sample(0.1);
        if((Vout_filter < valFloat[VoutMin_ID]) || (Vin_filter < (Vout_filter+off)))
          digitalWrite(dumploadPin, false);
        else
          digitalWrite(dumploadPin, true);

        unsigned long nowInMillis = millis()/TT;
        if (rpm_filter < 70 || nowInMillis > stopTimeInMillis)
          break;
      
        /*float t = (millis() - tmp_time) / (1000.*TT);
        if (t > 1)
        {
          int tmp_rpm = 60.*commutations/(t*48.);
          if (rpm_filter < 70)
            break;
          commutations = 0;
          tmp_time = millis();
        }*/
      }
      digitalWrite(dumploadPin, false);

      controlledStop();


      /*unsigned long lastNowInMillis = millis()/TT;
      long i=0;
      while (true)
      {
        unsigned long nowInMillis = millis()/TT;
        if (nowInMillis > lastNowInMillis)
        {
          ++i;
          if (i==10000)
            break;
          lastNowInMillis = nowInMillis;
        }
        digitalWrite(A_SD, HIGH);
        digitalWrite(B_SD, HIGH);
        digitalWrite(C_SD, HIGH);
        for (int j=0; j<i; ++j) {;}
        digitalWrite(A_SD, LOW);
        digitalWrite(B_SD, LOW);
        digitalWrite(C_SD, LOW);
        for (int j=10000; j>i; --j) {;}
      }*/

      digitalWrite(A_SD, HIGH);
      digitalWrite(B_SD, HIGH);
      digitalWrite(C_SD, HIGH);
      waitMS(2000);
      
      freeWheel();

    }
    acknowledge = true;
    break;
    
  case CMD_SAMPLE:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }

      freeWheel();

      unsigned long nextDump = millis() + 1000*TT;
      while (!getCommand())
      {
        sample();
        if (millis() > nextDump)
        {
          nextDump = millis() + 500*TT;
          Serial.print(F("V_in="));
          Serial.print(Vin_filter);
          Serial.print(F(" I_out="));
          Serial.print(Iout_filter);
          Serial.print(F(" V_out="));
          Serial.print(Vout_filter);
          Serial.print(F(" rpm="));
          Serial.println(rpm_filter);
        }
      }
    }
    break;

  case CMD_DIAGNOSTICS:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }

      int mode=0;
      sscanf((char*)(commandBuffer+1), "%d", &mode);

      freeWheel();

      while (!getCommand())
      {
        switch (mode)
        {
          case 0:
            digitalWrite(enableDriverPin, false);
            Serial.println(F("No mosfets conducting"));
            sampleAndDump(9, 500);
      
            /*Serial.println(F("High & Low mosfets conducting"));
            digitalWrite(enableDriverPin, false);
            analogWrite(highPin, 240);
            digitalWrite(enableDriverPin, true);
            sampleAndDump(9, 500);*/
      
            Serial.println(F("High mosfet conducting"));
            pwmHigh(240);
            sampleAndDump(9, 500);

            pwmDisable();
            Serial.println(F("No mosfets conducting"));
            sampleAndDump(9, 500);
      
            Serial.println(F("Low mosfet conducting"));
            pwmLow(15);
            sampleAndDump(9, 500);

            pwmDisable();
            Serial.println(F("No mosfets conducting"));
            sampleAndDump(9, 500);
            break;
  
          case 1:
            checkPhase("A", A_IN, A_SD);
            checkPhase("B", B_IN, B_SD);
            checkPhase("C", C_IN, C_SD);
          break;
  
          case 2:
            freeWheel();
            Serial.println(F("Buck converter low mosfet ON"));
            pwmLow(15);
            sampleAndDump(9, 500);
          
            /*Serial.println(F("Buck converter high mosfet ON"));
            digitalWrite(enableDriverPin, false);
            digitalWrite(highPin, true);
            analogWrite(enableDriverPin, 15);
            sampleAndDump(9, 500);*/
  
            Serial.println(F("A_low & B_low"));
            digitalWrite(A_SD, HIGH);
            digitalWrite(B_SD, HIGH);
            sampleAndDump(9, 500);
      
            freeWheel();
            Serial.println(F("A_low & C_low"));
            digitalWrite(A_SD, HIGH);
            digitalWrite(C_SD, HIGH);
            sampleAndDump(9, 500);
      
            freeWheel();
            Serial.println(F("B_low & C_low"));
            digitalWrite(B_SD, HIGH);
            digitalWrite(C_SD, HIGH);
            sampleAndDump(9, 500);
  
            Serial.println(F("Buck converter no mosfets ON"));
            pwmDisable();
            sampleAndDump(9, 500);
            break;
        }
      }
    }
    break;

  case CMD_ACKNOWLEDGE:
    acknowledge = true;
    break;
  case CMD_SPINUP:
    spinupNow = true;
    break;
  case CMD_ENABLE_WIND_SENSOR:
    sscanf((char*)(commandBuffer+1), "%d", &enableWindSensor);
    break;
  case CMD_SIMRUN:
    {
      sscanf((char*)(commandBuffer+1), "%d", &simRun);
    }
    break;

  case CMD_PRINTSETTINGS:
    {
      Serial.println(F("SETTINGS:"));
      Serial.print(F("VoutMax")); Serial.println(valFloat[VoutMax_ID]);
      Serial.print(F("VinMax")); Serial.println(valFloat[VinMax_ID]);
      Serial.print(F("VoutMin")); Serial.println(valFloat[VoutMin_ID]);
      Serial.print(F("IoutMax")); Serial.println(valFloat[IoutMax_ID]);
      Serial.print(F("TipSpeedRatio")); Serial.println(valFloat[TipSpeedRatio_ID]);
      Serial.print(F("Losses")); Serial.println(valFloat[Losses_ID]);
      Serial.print(F("WattMax")); Serial.println(valFloat[WattMax]);

      if (verboseLevel_old != 0)
        verboseLevel_old = verboseLevel;
      verboseLevel = 0;
    }
    break;

  case CMD_SET_SETTING:
    {
      char tmp1[5]="-1";
      char tmp2[5]="-1";
      sscanf((char*)(commandBuffer+1), "%s %s", tmp1, tmp2);
      int id = atoi(tmp1);
      float val = atof(tmp2);
      if (id==-1 || val==-1)
      {
        Serial.println(F("wrong parameters!"));
        break;
      }
      valFloat[id] = val;
    }
    break;

  case CMD_STORE_SETTINGS:
    eepromUpdateFloat16(valFloat, sizeof(valFloat)/4);
    Serial.println(F("done."));
    break;
    
  /*case CMD_LOAD_FACTORY_SETTINGS:
    if (state==RUNNING_CHARGE)
    {
      Serial.println(F("Command forbidden in charging state!"));
      break;
    }
    {
      for (int i=0; i<sizeof(powercurve)/4; ++i)
        powercurve[i] = pgm_read_float(factory_powercurve+i);
      //eepromUpdateFloat16(powercurve, sizeof(powercurve)/4);
    }
    break;
  case CMD_POWERCURVE:
    {
      Serial.println(F("POWER CURVE: (ID, voltage V, current A)"));
      for (int i=0; i<sizeof(powercurve)/4; ++i)
      {
        if (i%2 == 0)
        {
          Serial.print(i/2);
          Serial.print(F(","));
          Serial.print(powercurve[i]);
          Serial.print(F(","));
        }
        else
          Serial.println(powercurve[i]);
      }
      if (verboseLevel_old != 0)
        verboseLevel_old = verboseLevel;
      verboseLevel = 0;
    }
    break;
  case CMD_SET_POWER:
    {
      char id[3] = "-1";
      char voltage[5]="-1", current[5]="-1";
      sscanf((char*)(commandBuffer+1), "%s %s %s", id, voltage, current);
      if (atoi(id)==-1 || atof(voltage)==-1 || atof(current)==-1)
      {
        Serial.println(F("wrong parameters!"));
        break;
      }
      powercurve[atoi(id)*2] = atof(voltage);
      powercurve[atoi(id)*2+1] = atof(current);
    }
    break;
  case CMD_STORE_POWERCURVE:
    eepromUpdateFloat16(powercurve, sizeof(powercurve)/4);
    break;*/
  case CMD_WIND_PARAMS:
    {
      char val[4][5] = {"-1","-1","-1","-1"};
      sscanf((char*)(commandBuffer+1), "%s %s %s %s", val[0], val[1], val[2], val[3]);
      bool is_ok=true;
      for (byte i=0; i<4; ++i)
      {
        if (atoi(val[i]) == -1)
        {
          Serial.println(F("wrong parameters!"));
          is_ok=false;
          break;
        }
      }
      if (!is_ok)
        break;
      pressureThreshold = atof(val[0]);
      gust_spacing_min = atof(val[1]);
      gust_spacing_max = atof(val[2]);
      required_number_of_gusts = atof(val[3]);
      Serial.print(F("parameters: threshold="));
      Serial.print(pressureThreshold);
      Serial.print(F(" gust_min_spacing="));
      Serial.print(gust_spacing_min);
      Serial.print(F("s gust_max_spacing="));
      Serial.print(gust_spacing_max);
      Serial.print(F("s gust_count="));
      Serial.println(required_number_of_gusts);
    }
    break;
  case CMD_MIN_RUNTIME:
    {
      sscanf((char*)(commandBuffer+1), "%d", &minRunTimeInSeconds);
    }
    break;
  case CMD_SAMPLE_STARTUP_WIND:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }
      float pThres_old = pressureThreshold;
      pressureThreshold = 0.02;
      unsigned long oneSec = millis() + TT*1000;
      unsigned int sampleTimeInSeconds = 60;
      Serial.println(F("Sampling startup wind"));
      while (sampleTimeInSeconds > 0)
      {
        if (millis() > oneSec)
        {
          sampleTimeInSeconds--;
          oneSec += TT*1000;
        }
        if (getWindSpeedOK2())
        {
          Serial.print(F("threshold "));
          Serial.println(pressureThreshold);
          pressureThreshold += 0.02;
          sampleTimeInSeconds = 60;
        }
        if (getCommand())
        {
          pressureThreshold = pThres_old;
          commandBuffer[0] = CMD_IDLE;
          return true;
        }
      }
      pressureThreshold -= 0.02;
    }
    break;
  case CMD_COMMUTATE:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }

      int count=0;
      sscanf((char*)(commandBuffer+1), "%d", &count);
      if (count > 0)
      {
        makeAcknowledge(1, count, 2000);
        shortPhases();
      }
    }
    break;
  case CMD_HELP:
    printHelp();
    verboseLevel_old = verboseLevel;
    verboseLevel = 0;
    break;
  default:
    break;
  }

  commandBuffer[0] = CMD_IDLE;
  return false;
}
