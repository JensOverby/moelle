#ifndef HEADER
#define HEADER

#define A_SD 11
#define B_SD 5
#define C_SD 3
#define A_IN 12
#define B_IN 8
#define C_IN 4

#define ADC_VOLTAGE_IN  A0
#define ADC_CURRENT_OUT  A1
#define ADC_VOLTAGE_OUT A7
#define acs712VoltsPerAmp 0.066
#define voltageDividerVoltsPerVolt 0.089637248

#define TT 32
#define PHASE_THRESHOLD 50

#define highPin 9
#define dumploadPin 2
#define enableDriverPin 10
#define efficiency 0.9
#define pwmMin 30
#define pwmMax 242

#define VoutMax_ID 0
#define VinMax_ID 1
#define VoutMin_ID 2
#define IoutMax_ID 3
#define TipSpeedRatio_ID 4

/*PROGMEM const*/ float valFloat[] =  { 14.4, 40., 10., 20., 5. };

byte zc_event_val;

#define myConstrain(x, a, b)\
  if (x<a) x=a; else if (x>b) x=b;

#define whileMS(x)\
  unsigned long xxxTime = millis() + x*TT;\
  while (millis() < xxxTime)

#define whileS(x)\
  unsigned long xxxTime = millis() + x*long(TT)*1000;\
  while (millis() < xxxTime)

void sample(float k=0.02);
void eepromReadFloat16(float* pMem, int len, int address=0);
void makeAcknowledge(int directio=1, int commutations=12);
//bool getWindSpeedOK(bool verbose=false, float* std=NULL);
bool getWindSpeedOK2(bool verbose=false);
enum WindFeedbackType {FAIL,SUCCESS,COM_ERROR};
void windSensorFeedback2(WindFeedbackType value);
unsigned long zeroCrossSearch(unsigned long t_max, int hyst, unsigned long t_min=0);
void dump();
void breakNow(int voltageBegin=0, int voltageEnd=0, bool dump=false);
float readVcc();
float getExpectedPower(float rpm);

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

#endif
