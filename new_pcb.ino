#include "header.h"

//#define DEBUG

bool simRun = false;

enum{INIT,WAITING_FOR_CANCEL,WAITING,SPINUP,RUNNING_NO_CHARGE,RUNNING_CHARGE,STOPPED,BREAKING,TEST} state = INIT;

unsigned long spinupTime, runningTime=0;
unsigned int timeStampInSeconds = 0;

byte bldc_step = 0, motor_speed;
int com_error = 0;

float vcc = 5.;
bool shutDownFlag = false;
float Vin_filter=0, Vout_filter=0, Iin_filter=0;
#ifdef DEBUG
float Vout_filter_dbg=0;
#endif
float Iin_raw, Vin_raw, Vout_raw;
unsigned int min_sync_pwm = 255;
int verboseLevel = 2, verboseLevel_old=verboseLevel;

float duty_cycle = pwmMax;
float proportionalError=0;
//float sync_pwm_positive_offset = 0;

float pressureThreshold = 0.1;
byte gust_spacing_min = 2;
byte gust_spacing_max = 6;
byte required_number_of_gusts = 3;
unsigned int minRunTimeInSeconds = 30;

bool acknowledge = false;
bool spinupNow = false;
bool enableWindSensor = true;

int data_ptr = 0;
float powercurve[26];

float bemfPhaseA = 0;
unsigned long commutations = 0;

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
  TCCR1A =  0x21 | 0x81;
  TCCR1B = 0x01;
  
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

  if (verboseLevel) Serial.println(F("Power Curve:"));
  eepromReadFloat16(powercurve, sizeof(powercurve)/4);
}

bool synchronousBuck = true;
bool buckEnabled = false;

void enableBuck()
{
  duty_cycle = min_sync_pwm + 5;
  synchronousBuck = true;
  buckEnabled = true;
}

void disableBuck()
{
  digitalWrite(enableDriverPin, false);
  analogWrite(highPin, 0);
  buckEnabled = false;
}

void setBuckPWM(float& pwm)
{
  if (!buckEnabled)
    return;

//  myConstrain(pwm, /*pwmMin*/min_sync_pwm, pwmMax)

  if (synchronousBuck)
  {
    if (pwm < 150)
    {
      synchronousBuck = false;
      digitalWrite(enableDriverPin, false);
      if (verboseLevel) Serial.println("Asynchronous");
    }
    else
    {
      myConstrain(pwm, min_sync_pwm, pwmMax)
    }
    //if (pwm < min_sync_pwm)
    //{
    //  synchronousBuck = false;
    //  digitalWrite(enableDriverPin, false);
    //}
  }
  else
  {
    if (pwm > 160)
    {
      if (verboseLevel) Serial.println("Synchronous");
      synchronousBuck = true;
      digitalWrite(enableDriverPin, false);
      myConstrain(pwm, min_sync_pwm, pwmMax)
    }
    //else if (Iin_filter < 0.3)
    //{
    //  myConstrain(pwm, min_sync_pwm, pwmMax)
    //}
    //if (pwm >= min_sync_pwm+5)
    //{
    //  synchronousBuck = true;
    //  digitalWrite(enableDriverPin, false);
    //}
  }

  if (synchronousBuck)
  {
    analogWrite(highPin, pwm);
    digitalWrite(enableDriverPin, true);
  }
  else
  {
    digitalWrite(highPin, true);
    analogWrite(enableDriverPin, pwm);
  }
}

void loop()
{
  /*analogWrite(A_SD, 40);
  digitalWrite(A_IN, HIGH);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);

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
        //state = RUNNING_NO_CHARGE;
        //break;

        unsigned int valuePhaseB = analogRead(A2);
        bool verbose = (now - runningTime > 1000*TT);
        if (verbose)
        {
          runningTime = now;
          if (verboseLevel) Serial.print(F("INIT: measured speed value = "));
          if (verboseLevel) Serial.println(valuePhaseB);
        }
        //spinupNow = true;
        if (spinupNow || acknowledge || valuePhaseB > PHASE_THRESHOLD)
        {
          acknowledge = false;
          runningTime = now;
          while (isRunning(runningTime) && !spinupNow)
            waitMS(50);

          makeAcknowledge(1);

          if (enableWindSensor)
            windSpeedClear2();
  
          runningTime = millis();
          state = WAITING_FOR_CANCEL;
          if (verboseLevel) Serial.println(F("WAITING"));
        }

        getWindSpeedOK2();
        //waitMS(500);
      }
      break;
    case WAITING_FOR_CANCEL:
      {
        unsigned int valuePhaseB = analogRead(A2);
        if (acknowledge || valuePhaseB > PHASE_THRESHOLD)
        {
          acknowledge = false;
          runningTime = millis();
          while (isRunning(runningTime))
            waitMS(20);

          makeAcknowledge(-1);
          state = INIT;
          waitMS(1000);
          break;
        }
        
        unsigned int waitTimeInSeconds = (now - runningTime)/(1000*TT);
        if (waitTimeInSeconds > 15 || spinupNow)
        {
          state = WAITING;
          runningTime = 0;
        }

        getWindSpeedOK2();
        //waitMS(20);
      }
      break;
    case WAITING:
      {
        unsigned int valuePhaseB = analogRead(A2);
        if (acknowledge || valuePhaseB > PHASE_THRESHOLD)
        {
          acknowledge = false;
          runningTime = millis();
          while (isRunning(runningTime))
            waitMS(20);

          makeAcknowledge(-1);
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
        //byte debArray[600];
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
        byte com_state = 0;
        motor_speed = 50;
        float duty = motor_speed;
        float fac = 1;
        unsigned int tmp_counter = 0;
        byte pct;

        unsigned long T = 0;

        while (true)
        {
          bldc_move();
          bldc_step++;
          bldc_step %= 6;

          if (com_state < 2)
          {
            unsigned long zc = zeroCrossSearch(t, t/10);
            pct = (float(zc)/t) * 100;
            int error = pct - 90;
            t = t + fac*(t/6500.) * error;
          }
          else
          {
            t = zeroCrossSearch(1.2*t, t/40, true);
          }

          T += t;
          int tmp = t / 10;
          if (tmp < 100 || tmp>6000)
          {
            freeWheel();
            Serial.println(F("Commutation Error!"));
            com_error = 1;
            break;
          }


          switch (com_state)
          {
            case 0:
              if (pct == 100)
              {
                //if (verboseLevel) Serial.println(F("closed loop"));
                t *= 1.2;
                com_state=1;
              }
            break;
            case 1:
              tmp_counter++;
              if (tmp_counter == 48)
                com_state = 2;
              break;
            default:
              if (motor_speed<200)
              {
                duty += 1;
                motor_speed = duty;
              }
            break;
          }


          counter++;
          if (counter >= 800 || T > 5000000)
            break;
        }        
        
        freeWheel();
        ADCSRA = 135;
        ADCSRB = 0;
        BEMF_A_RISING();
        
        if (verboseLevel) Serial.print(F("t="));
        if (verboseLevel) Serial.println(t);
        if (verboseLevel) Serial.print(F("commutations="));
        if (verboseLevel) Serial.println(counter);

        //for (int i=0; i<sizeof(debArray); ++i)
        //  if (verboseLevel) Serial.println(debArray[i]);
        
        spinupTime = millis();
        runningTime = millis();

        if (verboseLevel) Serial.println(F("RUNNING, No charge"));
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
          if (verboseLevel) Serial.println("STOPPED");
          state = STOPPED;
          break;
        }

        if (Vin_filter > Vout_filter+2.)
        {
          if ((Vout_filter > VoutMin) && (Vout_filter < VoutMax))
          {
            enableBuck();
            setBuckPWM(duty_cycle);
            if (verboseLevel) Serial.println(F("State: Gate Driver Enabled. Circuit Ready"));
            whileMS(300)
              sample;
            
            state = RUNNING_CHARGE;
            timeStampInSeconds = nowInSeconds;
          }
        }

        dump();
      }
      break;
    case RUNNING_CHARGE:
      {
        sample();
        unsigned int nowInSeconds = now/(1000*TT);
        unsigned long nowInMillis = now/TT;

        bool ok = true;

        if (!isRunning(runningTime))
        {
          disableBuck();

          if (verboseLevel) Serial.println("STOPPED");
          state = STOPPED;
          break;
        }
        
        if (Iin_filter < -0.5)
        {
          disableBuck();

          if (verboseLevel) Serial.println(F("Iin_filter < -0.01"));
          whileMS(500)
            sample();
          ok = false;
        }

        if (Vin_filter <= (Vout_filter+0.1))
        {
          disableBuck();

          if (verboseLevel) Serial.print(F("Vin_filter <= 0.5+Vout_filter: "));
          if (verboseLevel) Serial.print(Vin_filter);
          if (verboseLevel) Serial.print(F(" <= 0.5+"));
          if (verboseLevel) Serial.println(Vout_filter);
          ok = false;
        }

        if (Vout_filter >= VoutMax && duty_cycle <= pwmMin)
        {
          disableBuck();

          if (verboseLevel) Serial.print(F("Vout_filter >= VoutMax: "));
          if (verboseLevel) Serial.print(Vout_filter);
          if (verboseLevel) Serial.print(F(" >= "));
          if (verboseLevel) Serial.println(VoutMax);
          ok = false;
        }
  
        if (Vout_filter < VoutMin && duty_cycle >= pwmMax)
        {
          disableBuck();

          if (verboseLevel) Serial.print(F("Vout_filter < VoutMin: "));
          if (verboseLevel) Serial.print(Vout_filter);
          if (verboseLevel) Serial.print(F(" < "));
          if (verboseLevel) Serial.println(VoutMin);
          ok = false;
        }
        
        if (Vin_filter > VinMax && duty_cycle >= pwmMax)
        {
          if (verboseLevel) Serial.println(F("Vin_filter > VinMax: BREAKING"));
          breakNow();
        }
        
        if (shutDownFlag)
        {
          if (Iin_filter < 0.06 /*|| Vin_filter < Vout_filter*/)
          {
            if (nowInSeconds > timeStampInSeconds+8)
            {
              disableBuck();

              if (verboseLevel) Serial.println(F("I < 0.04 timeout"));

              // Give voltage time to rise by 1 sec delay
              whileMS(1000)
                sample();

              ok = false;
            }
          }
          else
          {
            shutDownFlag = false;
            timeStampInSeconds = nowInSeconds;
          }
        }
        else if (Iin_filter < 0.06 /*|| Vin_filter < Vout_filter*/)
        {
          if (nowInSeconds > timeStampInSeconds+10)
          {
            timeStampInSeconds = nowInSeconds;
            shutDownFlag = true;
            if (verboseLevel) Serial.print(F("shutdown timestamp at "));
            if (verboseLevel) Serial.println(timeStampInSeconds);
          }
        }
  
        if (!ok)
        {
          state = RUNNING_NO_CHARGE;
          shutDownFlag = false;
          if (verboseLevel) Serial.println(F("State: No charge"));
        }
        /*else
        {
          duty_cycle = min_sync_pwm + simMargin;
          myConstrain(duty_cycle, pwmMin, pwmMax)
          setBuckPWM(duty_cycle);
        }
        else
        {
          if (nowInMillis > pwmUpdateTime)
          {
            // Real real-time
            float targetI;
            if (Vin_filter < (Vout_filter+2.))
            {
              float I1 = getInterpolatedY_(Vout_filter+2., powercurve, &data_ptr);
              targetI = interpolate(Vout_filter, 0, Vout_filter+2., I1, Vin_filter);
            }
            else
              targetI = getInterpolatedY_(Vin_filter, powercurve, &data_ptr);
            
            proportionalError = Iin_filter - targetI/4.;
            float correctionFactor = exp(-1.*proportionalError);

            sync_pwm_positive_offset *= correctionFactor;
            myConstrain(sync_pwm_positive_offset, 1/10, 10);

            pwmUpdateTime = nowInMillis + 100;
          }
          
          if (nowInMillis > pwmRemainderUpdateTime)
          {
            float pwm_abs_offset = sync_pwm_positive_offset+pwmRemainder;
            int pwm_offset = int(pwm_abs_offset);
            pwmRemainder = (pwm_abs_offset-pwm_offset);
            duty_cycle = min_sync_pwm + pwm_offset;
            //myConstrain(duty_cycle, last_duty_cycle-1, last_duty_cycle+1)
            last_duty_cycle = duty_cycle;
            myConstrain(duty_cycle, min_sync_pwm, pwmMax)
            setBuckPWM(duty_cycle);
            pwmRemainderUpdateTime = nowInMillis + 10;
          }
          
          dumpCharging(nowInSeconds);
        }*/
        else
        {
          // Real real-time
          float targetI;
          if (Vin_filter < (Vout_filter+2.))
          {
            float I1 = getInterpolatedY(Vout_filter+2., powercurve, &data_ptr);
            targetI = interpolate(Vout_filter, 0, Vout_filter+2., I1, Vin_filter);
          }
          else
            targetI = getInterpolatedY(Vin_filter, powercurve, &data_ptr);
          
          proportionalError = Iin_filter - targetI/6.;
          float duty_update = 0.2 * proportionalError;
          //myConstrain(duty_update,-0.04,0.04);
          duty_cycle -= duty_update;

          setBuckPWM(duty_cycle);
          //sync_pwm_positive_offset = duty_cycle - min_sync_pwm;
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
          if (verboseLevel) Serial.println(F("commutation error -> no sensor adjust"));
        }
        else if (timeOfRunInSeconds < minRunTimeInSeconds)
        {
          if (verboseLevel) Serial.print(F("Up time < "));
          if (verboseLevel) Serial.print(minRunTimeInSeconds);
          if (verboseLevel) Serial.println(F("secs -> startup failed"));
          if (enableWindSensor)
            windSensorFeedback2(false);
        }
        else
        {
          if (verboseLevel) Serial.print(F("Up time > "));
          if (verboseLevel) Serial.print(minRunTimeInSeconds);
          if (verboseLevel) Serial.println(F("secs -> startup succeeded"));
          if (enableWindSensor)
            windSensorFeedback2(true);
        }
        
        if (verboseLevel) Serial.println(F("WAITING"));
        if (enableWindSensor)
          windSpeedClear2();
        runningTime = millis();
        state = WAITING_FOR_CANCEL;
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
