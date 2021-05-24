#include "header.h"

//#define DEBUG

bool simRun = false;

enum{INIT,WAITING,SPINUP,RUNNING_NO_CHARGE,RUNNING_CHARGE,STOPPED,BREAKING,TEST} state = INIT;

unsigned long spinupTime, runningTime=0;
unsigned int noPowerTimeStampInSeconds = 0;
unsigned int powerRiseTimeStampInSeconds = 0;


byte bldc_step = 0, motor_speed;
int com_error = 0;

float vcc = 5.;
bool shutDownFlag = false;
float Vin_filter=0, Vout_filter=0, Iout_filter=0, rpm_filter=0;
#ifdef DEBUG
float Vout_filter_dbg=0;
#endif
float Iout_raw, Vin_raw, Vout_raw;
unsigned int min_sync_pwm = 255;
int verboseLevel = 2, verboseLevel_old=verboseLevel;

unsigned int duty_cycle = pwmMax;
float proportionalError=0;

float pressureThreshold = 0.1;
byte gust_spacing_min = 2;
byte gust_spacing_max = 6;
byte required_number_of_gusts = 3;
unsigned int minRunTimeInSeconds = 30;

bool acknowledge = false;
bool spinupNow = false;
bool enableWindSensor = true;
bool buckEnabled = false;

bool Vout_too_high = false;
bool Vin_too_high = false;

float bemfPhaseA = 0;


void setup()
{
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);

  pinMode(A_SD, OUTPUT);
  pinMode(B_SD, OUTPUT);
  pinMode(C_SD, OUTPUT);

  pinMode(dumploadPin, OUTPUT);
  digitalWrite(dumploadPin, false);
  pinMode(highPin, OUTPUT);
  analogWrite(highPin, 0);
  pinMode(enableDriverPin, OUTPUT);
  digitalWrite(enableDriverPin, false);

  // For Phase-correct PWM of 31.250 kHz (prescale factor of 1)
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);

  // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
  //TCCR1A =  0x21 | 0x81;
  //TCCR1B = 0x01;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1A |= (1 << COM1B1);
  TCCR1B = (1 << CS10) | (1 << WGM13) | (1 << WGM12);
  ICR1 = 255;  //This should be my frequency select: F_out = (F_clk/(2*prescaler*(1+ICR1)) = 10 kHz
  //OCR1A = 64;  //This should set D to ((1 + OCR1A)/(1 + ICR1)) * 100 = 5.1%
  //OCR1B = 254;

  // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR2A =  0x81;
  TCCR2B = 0x01;

  // Analog comparator setting
  //ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt

  freeWheel();

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  if (verboseLevel) Serial.println(F("Initializing wind sensor"));
  enableWindSensor = initWindSensor2();

  waitMS(1000);

  vcc = readVcc();
  if (verboseLevel) Serial.print(F("vcc = "));
  if (verboseLevel) Serial.println(vcc);

  unsigned long stopTime = millis() + 1000*TT;
  while (millis() < stopTime)
    sample();
  if (verboseLevel) Serial.print(F("min_sync_pwm = "));
  if (verboseLevel) Serial.println(min_sync_pwm);

  //if (verboseLevel) Serial.println(F("Power Curve:"));
  eepromReadFloat16(valFloat, sizeof(valFloat)/4);
}

void enableBuck()
{
  duty_cycle = min_sync_pwm + 5;
  if (duty_cycle < 180)
    duty_cycle = 180;
  buckEnabled = true;
  digitalWrite(enableDriverPin, true);
  analogWrite(highPin, duty_cycle);
}

void disableBuck()
{
  digitalWrite(enableDriverPin, false);
  analogWrite(highPin, 0);
  buckEnabled = false;
}

/*void setBuckPWM(unsigned int& pwm)
{
  if (!buckEnabled)
    return;

  if (pwm < min_sync_pwm && rpm_filter < 220)
    pwm = min_sync_pwm;
  if (pwm > pwmMax)
    pwm = pwmMax;

  digitalWrite(enableDriverPin, true);
  analogWrite(highPin, pwm);
}*/

void loop()
{
  /*digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  analogWrite(B_SD, 40);
  digitalWrite(B_IN, HIGH);

  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);

  while (true)
    waitMS(1000);*/

  if (getCommand())
    execCommand();

  unsigned long now = millis();
  switch(state)
  {
    case INIT:
      {
        shortPhases();

        bool verbose = (now - runningTime > 1000*TT);
        if (verbose)
        {
          runningTime = now;
          if (verboseLevel) Serial.println(F("INIT STATE"));
        }

        if (spinupNow || acknowledge)
        {
          acknowledge = false;
          runningTime = now;
          while (isRunning(runningTime) && !spinupNow)
            waitMS(50);

          makeAcknowledge(1);

          if (enableWindSensor)
            windSpeedClear2();
  
          runningTime = millis();
          state = WAITING;
          if (verboseLevel) Serial.println(F("WAITING"));
        }

        getWindSpeedOK2();
        //waitMS(500);
      }
      break;

    case WAITING:
      {
        shortPhases();

        if (acknowledge)
        {
          acknowledge = false;
          runningTime = millis();
          while (isRunning(runningTime))
            waitMS(20);

          state = INIT;
          waitMS(1000);
          break;
        }

        bool verbose = (now - runningTime > 1000*TT);
        if (verbose)
          runningTime = now;

        if (spinupNow || !enableWindSensor || getWindSpeedOK2(verbose && verboseLevel))
        {
          spinupNow = false;
          state = SPINUP;
          if (verboseLevel) Serial.println(F("SPINUP"));
        }

        //waitMS(20);
      }
      break;
    case SPINUP:
      {
        //int8_t debArray[300];
        int sample = 4000;

        freeWheel();
        waitMS(1000);
        
        int counter = 0;
        com_error = 0;

        for (bldc_step=0; bldc_step<6; ++bldc_step)
        {
          for (motor_speed=10; motor_speed<40; motor_speed++)
          {
            bldc_move();
            waitMS(10);
          }
        }
        bldc_step = 0;

        spinupTime = millis();

        unsigned long t = 40000;
        unsigned long t_max = t;
        unsigned long t_min = 0;
        int error = 0;
        unsigned long zc = 0;
        byte com_state = 0;

        
        motor_speed = 60;
        float duty = motor_speed;
        float fac = 1;
        int pct = 0;
        int myc = 0;

        unsigned long T = 0;

        while (true)
        {
          bldc_move();
          bldc_step++;
          bldc_step %= 6;

          switch (com_state)
          {
            case 0:
              zc = zeroCrossSearch(t, t/10);
              pct = (float(zc)/t) * 100;
              error = pct - 90;
              t = t + fac*(t/6500.) * error;
              if (pct == 100)
              {
                myc++;
                if (myc>5)
                {
                  com_state=1;
                  t /= 0.87f; // around 90%
                  myc = 0;
                }
              }
              break;
            case 1: // Really narrow transition state
              t = zeroCrossSearch(1.05*t, t/40, 0.92*t);
              ++myc;
              duty += 1;
              motor_speed = duty;
              if (myc > 70)
                com_state=2;
              break;
              
            default:
              t = zeroCrossSearch(1.1*t, t/40, 0.85*t);
              if (motor_speed<230)
              {
                duty += 1;
                motor_speed = duty;
              }
              break;
          }

          T += t;
          int tmp = t / 10;

          if (tmp < 100 || tmp>6000)
          {
            freeWheel();
            Serial.print(F("Commutation Error! State="));
            Serial.println(com_state);
            com_error = 1;
            break;
          }

          /*if (counter < 300)
          {
            int val = t/10 - sample;
            if (val>125)
              val=125;
            if (val<-125)
              val=-125;
            sample += val;
            debArray[counter] = (int8_t)val;
          }*/

          counter++;
          if (counter >= 800 || T > 5000000)
            break;
        }        
        
        freeWheel();
        ADCSRA = 135;
        ADCSRB = 0;
        BEMF_A_RISING();

        /*for (int i=0; i<counter; ++i)
        {
          Serial.println(debArray[i]);
        }
        Serial.print("comstate = ");
        Serial.println(com_state);*/

        if (verboseLevel) Serial.print(F("t="));
        if (verboseLevel) Serial.println(t);
        if (verboseLevel) Serial.print(F("commutations="));
        if (verboseLevel) Serial.println(counter);

        spinupTime = millis();
        runningTime = millis();

        if (verboseLevel) Serial.println(F("State: RUNNING, No charge"));
        state = RUNNING_NO_CHARGE;

        vcc = readVcc();
        if (verboseLevel) Serial.print(F("Vcc = "));
        if (verboseLevel) Serial.println(vcc);
      }
      break;

    case RUNNING_NO_CHARGE:
      {
        //vcc = readVcc();
        unsigned int nowInSeconds = now/(1000*TT);
        sample();

        if (!isRunning(runningTime))
        {
          if (verboseLevel) Serial.println("State: STOPPED");
          state = STOPPED;
          break;
        }

        if (Vin_filter > Vout_filter+2.)
        {
          if ((Vout_filter > valFloat[VoutMin_ID]) && (Vout_filter < valFloat[VoutMax_ID]))
          {
            if (verboseLevel) Serial.println(F("State: Gate Driver Enabled. Circuit Ready"));
            enableBuck();
            whileMS(300)
              sample;
            
            state = RUNNING_CHARGE;
            noPowerTimeStampInSeconds = nowInSeconds;
            powerRiseTimeStampInSeconds = nowInSeconds;
          }
        }

        dump();
      }
      break;
    case RUNNING_CHARGE:
      {
        whileMS(10)
          sample();

        unsigned int nowInSeconds = now/(1000*TT);
        unsigned long nowInMillis = now/TT;

        bool ok = true;

        if (!isRunning(runningTime))
        {
          disableBuck();

          if (verboseLevel) Serial.println("State: STOPPED");
          state = STOPPED;
          digitalWrite(dumploadPin, false);
          break;
        }
        
        if (Iout_filter < -0.5)
        {
          disableBuck();

          if (verboseLevel) Serial.println(F("Iout_filter < -0.5"));
          whileMS(500)
            sample();
          ok = false;
        }

        if (Vin_filter <= (Vout_filter+0.1))
        {
          disableBuck();

          if (verboseLevel) Serial.print(F("Vin_filter <= 0.1+Vout_filter: "));
          if (verboseLevel) Serial.print(Vin_filter);
          if (verboseLevel) Serial.print(F(" <= 0.1+"));
          if (verboseLevel) Serial.println(Vout_filter);
          ok = false;
        }

        if (Vout_filter < valFloat[VoutMin_ID] && duty_cycle >= pwmMax)
        {
          disableBuck();

          if (verboseLevel) Serial.print(F("Vout_filter < VoutMin: "));
          if (verboseLevel) Serial.print(Vout_filter);
          if (verboseLevel) Serial.print(F(" < "));
          if (verboseLevel) Serial.println(valFloat[VoutMin_ID]);
          ok = false;
        }
        
        if (Vin_filter > valFloat[VinMax_ID] && duty_cycle >= pwmMax)
        {
          if (verboseLevel) Serial.println(F("Vin_filter > VinMax: BREAKING"));
          breakNow();
        }
        
        if (shutDownFlag)
        {
          if (Iout_filter < 0.08 /*|| Vin_filter < Vout_filter*/)
          {
            if (nowInSeconds > noPowerTimeStampInSeconds+8)
            {
              disableBuck();

              if (verboseLevel) Serial.println(F("I < 0.08 timeout"));

              // Give voltage time to rise by 1 sec delay
              whileMS(1000)
                sample();

              ok = false;
            }
          }
          else
          {
            shutDownFlag = false;
            noPowerTimeStampInSeconds = nowInSeconds;
          }
        }
        else if (Iout_filter < 0.08 /*|| Vin_filter < Vout_filter*/)
        {
          if (nowInSeconds > noPowerTimeStampInSeconds+10)
          {
            noPowerTimeStampInSeconds = nowInSeconds;
            shutDownFlag = true;
            if (verboseLevel) Serial.print(F("NoPower timestamp at "));
            if (verboseLevel) Serial.println(noPowerTimeStampInSeconds);
          }
        }
  
        if (!ok)
        {
          digitalWrite(dumploadPin, false);
          state = RUNNING_NO_CHARGE;
          shutDownFlag = false;
          if (verboseLevel) Serial.println(F("State: No charge"));
        }
        else
        {
          // Real real-time

          float duty_update;
          if (Vout_filter < valFloat[VoutMax_ID])
          {
            if (Vout_too_high)
            {
              digitalWrite(dumploadPin, false);
              Vout_too_high = false;
            }

            float powerExp = getExpectedPower(rpm_filter);
            float powerReal = Vout_filter * Iout_filter;

            duty_update = powerReal - powerExp;


            //if (powerReal < 10 && rpm_filter < 150 && duty_cycle > 190)
            //  duty_update = 1;
          }
          else
          {
            if (Vin_filter > 20)
            {
              digitalWrite(dumploadPin, true);
              Vout_too_high = true;
            }

            duty_update = Vout_filter - valFloat[VoutMax_ID];
          }

          if (Vin_filter > valFloat[VinMax_ID])
          {
            duty_update = -1;
            if (duty_cycle >= pwmMax)
            {
              digitalWrite(dumploadPin, true);
              Vin_too_high = true;
            }
          }
          else 
          {
            if (Vin_too_high)
            {
              digitalWrite(dumploadPin, false);
              Vin_too_high = false;
            }
          }


          //myConstrain(duty_update, -1, 1)
          if (duty_update > 0)
            duty_cycle--;
          else
            duty_cycle++;

          //setBuckPWM(duty_cycle);

          if (duty_cycle < min_sync_pwm)
            duty_cycle = min_sync_pwm;
          if (duty_cycle > pwmMax)
            duty_cycle = pwmMax;

          //if (duty_cycle < min_sync_pwm && rpm_filter < 220)
          //  duty_cycle = min_sync_pwm;
          //if (duty_cycle > pwmMax)
          //  duty_cycle = pwmMax;
          
          digitalWrite(enableDriverPin, true);
          analogWrite(highPin, duty_cycle);

        }

        dump();
        //dump(nowInSeconds);
      }
      break;

    case STOPPED:
      {
        unsigned int timeOfRunInSeconds = (runningTime - spinupTime)/(1000*TT);
        if (verboseLevel) Serial.print(F("Turbine running time: "));
        if (verboseLevel) Serial.print(timeOfRunInSeconds);
        if (verboseLevel) Serial.println(F(" secs"));

        if (com_error)
        {
          if (verboseLevel) Serial.println(F("commutation error"));
          if (enableWindSensor)
            windSensorFeedback2(COM_ERROR);
        }
        else if (timeOfRunInSeconds < minRunTimeInSeconds)
        {
          if (verboseLevel) Serial.print(F("Up time < "));
          if (verboseLevel) Serial.print(minRunTimeInSeconds);
          if (verboseLevel) Serial.println(F("secs -> startup failed"));
          if (enableWindSensor)
            windSensorFeedback2(FAIL);
        }
        else
        {
          if (verboseLevel) Serial.print(F("Up time > "));
          if (verboseLevel) Serial.print(minRunTimeInSeconds);
          if (verboseLevel) Serial.println(F("secs -> startup succeeded"));
          if (enableWindSensor)
            windSensorFeedback2(SUCCESS);
        }
        
        if (verboseLevel) Serial.println(F("WAITING"));
        if (enableWindSensor)
          windSpeedClear2();
        runningTime = millis();
        state = WAITING;
      }
      break;
    
    case TEST:
      {
        if (verboseLevel) Serial.println(F("TEST"));
        delay(1000);
      }
      break;
    default:
      {
        if (verboseLevel) Serial.println(F("default"));
        delay(1000);
      }
      break;
  }
}
