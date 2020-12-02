//#define DEBUG

#define NO_SPINUP_ERRORS

#define PWM_START_DUTY   50
#define PWM_STEADY_DUTY   100
#define COMMUTATION_CYCLES_PER_SEC 40
#define TT 32
#define PHASE_THRESHOLD 50

#define highPin 9
#define dumploadPin 13
#define enableDriverPin 10
#define efficiency 0.9
#define pwmMin 30
#define pwmMax 242
#define VoutMax 16.4
#define VinMax 53
#define VoutMin 10
#define IoutMax 20

#define myConstrain(x, a, b)\
  if (x<a) x=a; else if (x>b) x=b;

#define whileMs(x)\
  unsigned long xxxTime = millis() + x*TT;\
  while (millis() < xxxTime)

bool simRun = false;

enum{INIT,WAITING_FOR_CANCEL,WAITING,SPINUP,RUNNING_NO_CHARGE,RUNNING_CHARGE,STOPPED,BREAKING,TEST} state = INIT;

unsigned long spinupTime, runningTime=0;
unsigned int timeStampInSeconds = 0;

byte bldc_step = 0, motor_speed;
volatile bool toggle = false;

float vcc = 5.;
bool shutDownFlag = false;
float Vin_filter=0, Vout_filter=0, Iin_filter=0;
#ifdef DEBUG
float Vout_filter_dbg=0;
#endif
float Iin_raw, Vin_raw, Vout_raw;
unsigned int min_sync_pwm = 255;
int verboseLevel = 2;

float duty_cycle = pwmMax;
float proportionalError=0;
float sync_pwm_positive_offset = 0;

int rule_error = 0;

bool spinupNow = false;
bool enableWindSensor = true;

int data_ptr = 0;
/*PROGMEM*/ const float data[] = {0.04,60, 0.02,75, 0.01,115, 0.007,135, 0.005,170, 0.004,205};

float powercurve[26];

/*const float powercurve[] = {26, 17.29,
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

void waitMS(float stopT)
{
  stopT = millis() + stopT*TT;
  while (millis() < stopT);
}

void waitUnits(unsigned long stopT)
{
  stopT = millis() + stopT;
  while (millis() < stopT);
}

void setup()
{
  setup_startup();

  pinMode(highPin, OUTPUT);
  pinMode(dumploadPin, OUTPUT);
  pinMode(enableDriverPin, OUTPUT);

  // For Phase-correct PWM of 31.250 kHz (prescale factor of 1)
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);

  // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
  //TCCR1A = 0;
  TCCR1A =  0x21 | 0x81;
  TCCR1B = 0x01;
  
  // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
  //TCCR2A = 0;
  TCCR2A =  0x81;
  TCCR2B = 0x01;
  // Analog comparator setting
  ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt

  digitalWrite(enableDriverPin, false);
  analogWrite(highPin, 0);
  digitalWrite(dumploadPin, false);

  freeWheel();

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  Serial.println(F("Initializing wind sensor"));
  enableWindSensor = initWindSensor();

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

  waitMS(1000);

  vcc = readVcc();
  Serial.print(F("vcc = "));
  Serial.println(vcc);

  //AdcIntSetup();

  unsigned long stopTime = millis() + 1000*TT;
  while (millis() < stopTime)
    sample();
  Serial.print(F("min_sync_pwm = "));
  Serial.println(min_sync_pwm);

  Serial.println(F("Power Curve:"));
  eepromReadFloat16(powercurve, sizeof(powercurve)/4);
}

bool synchronousBuck = true;
bool buckEnabled = false;

void enableBuck()
{
  duty_cycle = min_sync_pwm + 5;
  synchronousBuck = true;
  buckEnabled = true;
  //sync_pwm_positive_offset = 2.5;
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

  myConstrain(pwm, /*pwmMin*/min_sync_pwm, pwmMax)

  /*if (synchronousBuck)
  {
    if (Iin_filter > 0.5)
    {
      synchronousBuck = false;
      digitalWrite(enableDriverPin, false);
      Serial.println("Asynchronous");
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
    if (Iin_filter < 0.15)
    {
      Serial.println("Synchronous");
      synchronousBuck = true;
      digitalWrite(enableDriverPin, false);
      myConstrain(pwm, min_sync_pwm, pwmMax)
    }
    else if (Iin_filter < 0.3)
    {
      myConstrain(pwm, min_sync_pwm, pwmMax)
    }
    //if (pwm >= min_sync_pwm+5)
    //{
    //  synchronousBuck = true;
    //  digitalWrite(enableDriverPin, false);
    //}
  }*/

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
  /*digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  analogWrite(B_SD, motor_speed);
  digitalWrite(B_IN, HIGH);

  digitalWrite(C_SD, LOW);
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
        Serial.print(F("INIT: measured speed value = "));
        Serial.println(valuePhaseB);
        if (spinupNow || valuePhaseB > PHASE_THRESHOLD)
        {
          runningTime = now;
          while (isRunning(runningTime))
            waitMS(50);

          makeAcknowledge(1);

          if (enableWindSensor)
            windSpeedClear();
  
          runningTime = millis();
          state = WAITING_FOR_CANCEL;
          Serial.println(F("WAITING"));
        }
        waitMS(500);
      }
      break;
    case WAITING_FOR_CANCEL:
      {
        unsigned int waitTimeInSeconds = (now - runningTime)/(1000*TT);
        if (waitTimeInSeconds > 15)
        {
          state = WAITING;
          runningTime = now;
        }

        unsigned int valuePhaseB = analogRead(A2);
        if (valuePhaseB > PHASE_THRESHOLD)
        {
          runningTime = millis();
          while (isRunning(runningTime))
            waitMS(50);

          makeAcknowledge(-1);
          state = INIT;
        }

        waitMS(20);
      }
      break;
    case WAITING:
      {
        if (spinupNow || !enableWindSensor || getWindSpeedOK())
        {
          spinupNow = false;
          state = SPINUP;
          Serial.println(F("SPINUP"));
        }

        unsigned int valuePhaseB = analogRead(A2);
        if (valuePhaseB > PHASE_THRESHOLD)
        {
          runningTime = millis();
          while (isRunning(runningTime))
            waitMS(50);

          makeAcknowledge(-1);
          state = INIT;
        }

        waitMS(20);
      }
      break;
    case SPINUP:
      {
        waitMS(300);
        makeAcknowledge(1);

        spinupTime = millis();

        const float commutation_angle = 3.14159/3;
        float dT = 0.04;
        data_ptr = 0;

        while (true)
        {
          float angular_speed = commutation_angle / dT;
          angular_speed += 20.0*dT;//50.0 * dT;
          dT = commutation_angle / angular_speed;
  
          motor_speed = getInterpolatedY(dT, data, &data_ptr);

          bldc_move();
          bldc_step++;
          bldc_step %= 6;

          waitMS(dT*1000.);

          if (dT <= 0.01)
            break;
        }        

        int8_t debArray[120];
        unsigned int old_dt = 127;
        unsigned int old_comTime = 255;
        
        byte errorCode = 0;
        unsigned int rule1_counter=0, rule2_counter=-1;

        unsigned int globalMaxComTime = dT*1000*TT;
        unsigned int comTime = globalMaxComTime;
        unsigned long t_old = millis();

        motor_speed = PWM_STEADY_DUTY;
        unsigned int turnCounter=0, turnCounterOld = 0;
        unsigned long nextCheckTime = millis() + 1000*TT;
        bool cancelled = false;
        toggle = true;
        bldc_step += 1;
        bldc_step %= 6;
        byte count_ = 0;
        count_ = 0;
        ACSR |= 0x08;                    // Enable analog comparator interrupt
        waitUnits(comTime/4); //delayMicroseconds(comTime/4);
        toggle = false;
        while(true)
        {
          unsigned int maxComTime = comTime + comTime/2 + 1;
          if (maxComTime > globalMaxComTime)
            maxComTime = globalMaxComTime;
          unsigned long t;
          unsigned int dt;
          bool forced = false;

          while (true)
          {
            t = millis();
            dt = t - t_old;

            noInterrupts();
            if (dt > maxComTime)
            {
              if (!toggle)
              {
                toggle = true;
                bldc_move();
                bldc_step++;
                bldc_step %= 6;
                forced = true;
              }
            }
            interrupts();
            
            if (toggle)
              break;
          }
          if (motor_speed < 200)
            motor_speed += 2;

          // BEGIN, rule based error
          rule_error = 0;
          if (!forced)
          {
            comTime = 0.1*float(dt) + 0.9*float(comTime);

            if (dt < 5*TT && rule1_counter < 40)
            {
              if (rule2_counter != -1)
                rule_error = 1; // More than one "Lows" detected
              else
                rule2_counter = 0; // Counter for number of forcings after "Low" detection
            }
          }
          else
          {
            if (rule2_counter != -1)
            {
              rule2_counter++;
              if (rule2_counter > 7)
                rule_error = 2; // More than 7 forcings detected;
            }
            if (rule1_counter == 6 || rule1_counter == 7)
              rule_error = 3; // commutation 7 or 8 must not be forced
          }

          if (rule1_counter > 5 && rule2_counter == -1)
            rule_error = 4; // "Low" not detected in first 6 commutations
          rule1_counter++;
#ifndef NO_SPINUP_ERRORS
          if (rule_error)
            break;
#endif
          // END, rule based error

          if (count_ < sizeof(debArray))
          {
            int diff = dt - old_dt;
            myConstrain(diff, -127, 127)
            old_dt += diff;
            debArray[count_] = diff;

            diff = comTime - old_comTime;
            myConstrain(diff, -127, 127)
            old_comTime += diff;
            debArray[count_+1] = diff;
            
            count_ += 2;
          }

          t_old = t;

          // Prevent BEMF debounce
          waitUnits(comTime/3); //delayMicroseconds(comTime/4);
          toggle = false;
          turnCounter++;

          runningTime = millis();
          if (runningTime > nextCheckTime)
          {
            if (turnCounter > COMMUTATION_CYCLES_PER_SEC*6)
              break;
            if (turnCounter <= turnCounterOld)
            {
              cancelled = true;
              break;
            }
            turnCounterOld = turnCounter;
            turnCounter = 0;
            nextCheckTime += 1000*TT;
          }
        }

        ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt
        freeWheel();
        waitMS(100); //delay(100);

        Serial.print(count_/2);
        Serial.println(F(" data points logged"));
        
        old_dt = 127;
        old_comTime = 255;
        for (count_=0;count_<sizeof(debArray);count_+=2)
        {
          sample();
          old_dt += debArray[count_];
          Serial.print(old_dt);
          Serial.print(F(","));
          old_comTime += debArray[count_+1];
          Serial.println(old_comTime);
        }

        Serial.print(F("cycles/sec: "));
        Serial.println(turnCounter/6);
        if (cancelled)
          Serial.println(F("CANCEL SPINUP"));
        runningTime = millis();

        if (rule_error)
        {
          Serial.print(F("Startup Error: "));
          switch(rule_error)
          {
            case 1:
              Serial.println(F("More than one Lows detected"));
              break;
            case 2:
              Serial.println(F("More than 7 forcings detected"));
              break;
            case 3:
              Serial.println(F("commutation 7 or 8 must not be forced"));
              break;
            case 4:
              Serial.println(F("Low not detected in first 6 commutations"));
              break;
            default:
              break;
          }
        }
        else
          Serial.println(F("RUNNING, No charge"));

        data_ptr = 0;
        state = RUNNING_NO_CHARGE;

        vcc = readVcc();
        Serial.print(F("Vcc = "));
        Serial.println(vcc);
        // DELETE THIS
        //state = WAITING_FOR_CANCEL;

      }
      break;
    case RUNNING_NO_CHARGE:
      {
        vcc = readVcc();
        unsigned int nowInSeconds = now/(1000*TT);
        sample();

        if (!isRunning(runningTime))
        {
          Serial.println("STOPPED");
          state = STOPPED;
          break;
        }

        if (Vin_filter > Vout_filter+2.)
        {
          if ((Vout_filter > VoutMin) && (Vout_filter < VoutMax))
          {
            enableBuck();
            setBuckPWM(duty_cycle);
            Serial.println(F("State: Gate Driver Enabled. Circuit Ready"));
            whileMs(300)
              sample;
            
            state = RUNNING_CHARGE;
            timeStampInSeconds = nowInSeconds;
          }
        }

        dump(nowInSeconds);
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

          Serial.println("STOPPED");
          state = STOPPED;
          break;
        }
        
        if (Iin_filter < -0.5)
        {
          disableBuck();

          Serial.println(F("Iin_filter < -0.01"));
          whileMs(500)
            sample();
          ok = false;
        }

        if (Vin_filter <= (Vout_filter+0.1))
        {
          disableBuck();

          Serial.print(F("Vin_filter <= 0.5+Vout_filter: "));
          Serial.print(Vin_filter);
          Serial.print(F(" <= 0.5+"));
          Serial.println(Vout_filter);
          ok = false;
        }

        if (Vout_filter >= VoutMax && duty_cycle <= pwmMin)
        {
          disableBuck();

          Serial.print(F("Vout_filter >= VoutMax: "));
          Serial.print(Vout_filter);
          Serial.print(F(" >= "));
          Serial.println(VoutMax);
          ok = false;
        }
  
        if (Vout_filter < VoutMin && duty_cycle >= pwmMax)
        {
          disableBuck();

          Serial.print(F("Vout_filter < VoutMin: "));
          Serial.print(Vout_filter);
          Serial.print(F(" < "));
          Serial.println(VoutMin);
          ok = false;
        }
        
        if (Vin_filter > VinMax && duty_cycle >= pwmMax)
        {
          Serial.println(F("Vin_filter > VinMax: BREAKING to 16V"));
          breakNow(16);
        }
        
        if (shutDownFlag)
        {
          if (Iin_filter < 0.06 /*|| Vin_filter < Vout_filter*/)
          {
            if (nowInSeconds > timeStampInSeconds+8)
            {
              disableBuck();

              Serial.println(F("I < 0.04 timeout"));

              // Give voltage time to rise by 1 sec delay
              whileMs(1000)
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
            Serial.print(F("shutdown timestamp at "));
            Serial.println(timeStampInSeconds);
          }
        }
  
        if (!ok)
        {
          state = RUNNING_NO_CHARGE;
          shutDownFlag = false;
          Serial.println(F("State: No charge"));
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
            float I1 = getInterpolatedY_(Vout_filter+2., powercurve, &data_ptr);
            targetI = interpolate(Vout_filter, 0, Vout_filter+2., I1, Vin_filter);
          }
          else
            targetI = getInterpolatedY_(Vin_filter, powercurve, &data_ptr);
          
          proportionalError = Iin_filter - targetI/4.;
          float duty_update = 0.2 * proportionalError;
          duty_cycle -= duty_update;

          setBuckPWM(duty_cycle);
          sync_pwm_positive_offset = duty_cycle - min_sync_pwm;
        }

        dumpCharging(nowInSeconds);
        //dump(nowInSeconds);
      }
      break;
    case STOPPED:
      {
        unsigned int timeOfRunInSeconds = (runningTime - spinupTime)/(1000*TT);
        Serial.print(F("Turbine running time: "));
        Serial.print(timeOfRunInSeconds);
        Serial.println(F(" secs"));

        if (rule_error)
          Serial.println(F("commutation error -> no sensor adjust"));
        else if (timeOfRunInSeconds < 2*60)
        {
          Serial.println(F("Up time < 2 min -> startup failed"));
          if (enableWindSensor)
            windSensorFeedback(false);
        }
        else if (timeOfRunInSeconds > 3*60)
        {
          Serial.println(F("Up time > 3 min -> startup succeeded"));
          if (enableWindSensor)
            windSensorFeedback(true);
        }
        
        Serial.println(F("WAITING"));
        if (enableWindSensor)
          windSpeedClear();
        runningTime = millis();
        state = WAITING_FOR_CANCEL;
      }
      break;
    
    case TEST:
      {
        Serial.println(F("TEST"));
        delay(1000);
      }
      break;
    default:
      {
        Serial.println(F("default"));
        delay(1000);
      }
      break;
  }
}
