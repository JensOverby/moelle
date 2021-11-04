int lastCount;

// Analog comparator ISR
/*ISR (ANALOG_COMP_vect)
{
  if (toggle)
    return;
  toggle = true;
}*/

void bldc_move()        // BLDC motor commutation function
{
  switch(bldc_step){
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}

void BEMF_A_RISING()
{
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  //ACSR |= 0x03;            // Set interrupt on rising edge
  ACSR = 0x00;
  zc_event_val = 0x20;
}

void BEMF_A_FALLING()
{
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  //ACSR &= ~0x01;           // Set interrupt on falling edge
  ACSR = 0x00;
  zc_event_val = 0x0;
}

void BEMF_B_RISING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  //ACSR |= 0x03;
  ACSR = 0x00;
  zc_event_val = 0x20;
}

void BEMF_B_FALLING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  //ACSR &= ~0x01;
  ACSR = 0x00;
  zc_event_val = 0x0;
}

void BEMF_C_RISING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  //ACSR |= 0x03;
  ACSR = 0x00;
  zc_event_val = 0x20;
}

void BEMF_C_FALLING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  //ACSR &= ~0x01;
  ACSR = 0x00;
  zc_event_val = 0x0;
}

void AH_BL()
{
  analogWrite(A_SD, motor_speed);
  digitalWrite(A_IN, HIGH);
  
  digitalWrite(B_SD, HIGH);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void AH_CL()
{
  analogWrite(A_SD, motor_speed);
  digitalWrite(A_IN, HIGH);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);
}

void BH_CL()
{
  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  analogWrite(B_SD, motor_speed);
  digitalWrite(B_IN, HIGH);
  
  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);
}

void BH_AL()
{
  digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  analogWrite(B_SD, motor_speed);
  digitalWrite(B_IN, HIGH);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void CH_AL()
{
  digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  analogWrite(C_SD, motor_speed);
  digitalWrite(C_IN, HIGH);
}

void CH_BL()
{
  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, HIGH);
  digitalWrite(B_IN, LOW);
  
  analogWrite(C_SD, motor_speed);
  digitalWrite(C_IN, HIGH);
}

void freeWheel()
{
  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void controlledStop()
{
/*  for (int h=1; h<10; h++)
  {
    for (int i=0; i<100; i++)
    {
      digitalWrite(A_SD, HIGH);
      digitalWrite(B_SD, HIGH);
      digitalWrite(C_SD, HIGH);
      waitMS(h);
      digitalWrite(A_SD, LOW);
      digitalWrite(B_SD, LOW);
      digitalWrite(C_SD, LOW);
      waitMS((10-h));
    }
  }*/
}

void shortPhases()
{
  freeWheel();
/*  
  digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, HIGH);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);*/
}

void makeAcknowledge(int directio, int commutations, int delayTime)
{
#ifdef BENCH_TEST
  Serial.println(F("BENCH TEST"));
  return;
#endif
  waitMS(300);
  
  for (int j = directio==1? 0 : commutations-1; j>-1 && j<commutations; j += directio)
  {
    bldc_step = j % 6;
    if (delayTime != 1)
    {
      Serial.print(F("Step = "));
      Serial.println(bldc_step);
    }
    for (motor_speed=10; motor_speed<50; motor_speed++)
    {
      bldc_move();
      waitMS(10);
    }
    waitMS(delayTime);
  }
  bldc_step = (bldc_step+directio) % 6;

  freeWheel();
}

bool isRunning(unsigned long& refTime)
{
  unsigned int valuePhaseB = analogRead(A2);
  unsigned long now = millis();
  if (valuePhaseB > PHASE_THRESHOLD || simRun)
  {
    refTime = now;
    return true;
  }
  int count = (now-refTime)/(1000*TT);
  if (count > 10)
    return false;
  if (count > 2)
  {
    if (count != lastCount)
    {
      if (verboseLevel) Serial.print(F("countdown = "));
      if (verboseLevel) Serial.println(25 - count);
      lastCount = count;
    }
  }
  return true;
}

/*    This function:
 *     t           : is time duration of a single commutation
 *     zc_event_val: a value that toggles for each commutation. Rising or Falling edge.

 */
unsigned long zeroCrossSearch(unsigned long t_max, int hyst, unsigned long t_min)
{
  bool zero_cross_terminate = (t_min != 0);
  unsigned long i, zc=0;
  int c = -hyst;
  for (i=0; i<t_max; ++i)
  {
    if ((ACSR & 0x20) == zc_event_val)
      ++c;
    else
    {
      if (c > -hyst)
        --c;
    }
    if (c > 0 && zc == 0 && i > t_min)
    {
      zc = i;
      if (zero_cross_terminate)
        break;
    }
  }

  if (zc == 0)
    return i;
  return zc;
}
