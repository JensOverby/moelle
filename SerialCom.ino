#include <EEPROM.h>

enum WorkSpcRole {CMD_IDLE=0,
                  CMD_HELP=1,
                  CMD_VERBOSE=2,
                  CMD_DUTY=3,
                  CMD_DUMPLOAD=4,
                  CMD_SPINUP=5,
                  CMD_ENABLE_WIND_SENSOR=6,
                  CMD_SAMPLE=7,
                  CMD_SIMRUN=8,
                  CMD_FACTORY_RESET=9};
                  //CMD_CURFAC=11};
char commandBuffer[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static unsigned int bytesReceived = 0;
static unsigned int bytesExpected = 0;
static unsigned int hashValue = 0;

void printHelp()
{
  Serial.println(F("Commands:"));
  Serial.println(F("-----------------"));
  Serial.println(F("help            : prints this message"));
  Serial.println(F("spinup          : spin up turbine"));
  Serial.println(F("windsense <1/0> : wind sensor on/off"));
  Serial.println(F("sample          : sample v_in,i_in,v_out"));
  Serial.println(F("verbose <1/0>   : verbose on/off"));
  Serial.println(F("duty <value>    : set duty cycle (0-255)"));
  Serial.println(F("dumpload <V-begin> <V-end> <duty>"));
  Serial.println(F("                : dump to dump resistor"));
  Serial.println(F("simrun <1/0>    : simulate turbine running"));
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
            Serial.print(F("hash value = "));
            Serial.println(hashValue);
          }
          c = CMD_IDLE;
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
        /*case 27735: // regulate
          bytesExpected = -1;
          c = CMD_REGULATE;
          circuitState = INIT;
          break;*/
        case 41053: // dumpload
          bytesExpected = -1;
          c = CMD_DUMPLOAD;
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
        case 58025:
          bytesExpected = -1;
          c = CMD_SIMRUN;
          break;
        case 9186:
          bytesExpected = -1;
          c = CMD_FACTORY_RESET;
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

PROGMEM const float factory_powercurve[] =  {26, 17.29,
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
                                             14, 0.85};

void eepromReadFloat16(float* pMem, int len, int address=0)
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
    Serial.print(",");*/
    if (i%2 == 0)
    {
      Serial.print(pMem[i]);
      Serial.print(F(","));
    }
    else
      Serial.println(pMem[i]);
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

    if (i%2 == 0)
    {
      Serial.print(pMem[i]);
      Serial.print(F(","));
    }
    else
      Serial.println(pMem[i]);
    
    /*Serial.print(iVal);
    Serial.print(",");
    Serial.print(value[0]);
    Serial.print(",");
    Serial.println(value[1]);*/
  }
}

void execCommand()
{
  switch (commandBuffer[0])
  {
  case CMD_VERBOSE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &verboseLevel);
    }
    break;
  case CMD_DUTY:
    {
      digitalWrite(enableDriverPin, true);
      duty_cycle = pwmMin;
      int counter = 0;
      int duty;
      sscanf((char*)(commandBuffer+1), "%d", &duty);
      while (!getCommand())
      {
        if (duty>duty_cycle)
          duty_cycle++;
        else if (duty<duty_cycle)
          duty_cycle--;

        analogWrite(highPin, duty_cycle);
        sample();

        counter++;
        if (counter>100)
        {
          counter=0;
          dump();
        }
        waitMS(1);
      }
      digitalWrite(enableDriverPin, false);
    }
    break;
  case CMD_DUMPLOAD:
    {
      int voltageBegin = -1, voltageEnd = -1, duty = -1;
      sscanf((char*)(commandBuffer+1), "%d %d %d", &voltageBegin, &voltageEnd, &duty);
      if (voltageBegin == -1 || voltageEnd == -1 || duty == -1)
      {
        Serial.println(F("wrong parameters!"));
        break;
      }

      breakNow(voltageBegin, voltageEnd, duty, 0, true);
    }
    break;
  case CMD_SAMPLE:
    {
      unsigned long nextDump = 0;
      while (!getCommand())
      {
        sample();
        if (millis() > nextDump)
        {
          nextDump = millis() + 500*TT;
          Serial.print(F("V_in="));
          Serial.print(Vin_filter);
          Serial.print(F(" I_in="));
          Serial.print(Iin_filter);
          Serial.print(F(" V_out="));
          Serial.println(Vout_filter);
        }
      }
    }
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
  case CMD_FACTORY_RESET:
    {
      for (int i=0; i<sizeof(powercurve)/4; ++i)
        powercurve[i] = pgm_read_float(factory_powercurve+i);
      eepromUpdateFloat16(powercurve, sizeof(powercurve)/4);
    }
    break;
  case CMD_HELP:
    printHelp();
    break;
  default:
    break;
  }

  commandBuffer[0] = CMD_IDLE;
}
