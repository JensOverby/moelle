#define A_SD 11
#define B_SD 5
#define C_SD 3
#define A_IN 12
#define B_IN 8
#define C_IN 4

//#define DEBUG_PIN 13

//unsigned int i;
int lastCount;

void setup_startup()
{
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);

  pinMode(A_SD, OUTPUT);
  pinMode(B_SD, OUTPUT);
  pinMode(C_SD, OUTPUT);

  motor_speed = PWM_START_DUTY;
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect)
{
  if (toggle)
    return;
  toggle = true;

  bldc_move();
  bldc_step++;
  bldc_step %= 6;
}

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
  ACSR |= 0x03;            // Set interrupt on rising edge
}

void BEMF_A_FALLING()
{
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR &= ~0x01;           // Set interrupt on falling edge
}

void BEMF_B_RISING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR |= 0x03;
}

void BEMF_B_FALLING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01;
}

void BEMF_C_RISING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR |= 0x03;
}

void BEMF_C_FALLING()
{
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR &= ~0x01;
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
  ADCSRA = 135;
  ADCSRB = 0;

  digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);
  
  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);
}

void makeAcknowledge(int directio=1, int commutations=12)
{
  waitMS(300);
  
  for (int j = directio==1? 1 : commutations; j>0 && j<=commutations; j += directio)
  {
    bldc_step = j % 6;
    for (motor_speed=10; motor_speed<40; motor_speed++)
    {
      bldc_move();
      waitMS(10);
    }
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
  if (count > 25)
    return false;
  if (count > 2)
  {
    if (count != lastCount)
    {
      Serial.print(F("countdown = "));
      Serial.println(25 - count);
      lastCount = count;
    }
  }
  return true;
}

float interpolate(float x0, float y0, float x1, float y1, float x) { return y0 + (x-x0)/(x1-x0) * (y1-y0); }
float getInterpolatedY(float x, float* pData, int* data_ptr)
{
  if (x > pgm_read_float(pData))
    return pgm_read_float(pData+1);
  int data_sz = sizeof(pData)/4;
  if (x <= pgm_read_float(pData+data_sz-2))
    return pgm_read_float(pData+data_sz-1);
  while (pgm_read_float(pData+(*data_ptr)*2+2) > x)
    (*data_ptr)++;
  while (pgm_read_float(pData+(*data_ptr)*2) <= x)
    (*data_ptr)--;
  float y = interpolate(pgm_read_float(pData+(*data_ptr)*2),pgm_read_float(pData+(*data_ptr)*2+1),pgm_read_float(pData+(*data_ptr)*2+2),pgm_read_float(pData+(*data_ptr)*2+3),x);
  return y;
}

float getInterpolatedY_(float x, float* pData, int* data_ptr)
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
