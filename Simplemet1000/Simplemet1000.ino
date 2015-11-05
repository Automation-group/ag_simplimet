#include "T6963.h"
#include "gfxdata.h"
#include "Times_New_Roman__14.h"

void setPwmFrequency(int pin, int divisor);

T6963 LCD(240,128,6,32);// 240x64 Pixel and 6x8 Font

const char keyboardPins[] = {A8, A9, A10, A11, A12, A13, A14, A15};
const char menuKeyMask = 0x01;
const char pin1KeyMask = 0x02;
const char pin2KeyMask = 0x04;
const char pin3KeyMask = 0x08;
const char downKeyMask = 0x10;
const char upKeyMask = 0x20;
const char offKeyMask = 0x40;
const char onKeyMask = 0x80;

const int motorPressUpPin = 2;
const int motorPressDownPin = 3;

const int furnacePin = 16; 
const int waterValvePin = 14;
const int isClosedPin = 18;

const int PID_HISTORY_SIZE = 20;
int maxOutputPower = 20;

class PID {
  public:
    int t_Integ;
    int t_Diff;
    int Coeff;
    int Setting; // уставка
    int HistoryValues[PID_HISTORY_SIZE];
    unsigned char Index;    
  public:
    PID() {
      t_Integ = 32000;
      t_Diff = 4;
      Index = 0;
      Coeff = 50;
      Setting = 0;

      unsigned char i;
      for(i=0;i<PID_HISTORY_SIZE;i++) {
        HistoryValues[i] = 0;
      }      
    }
    
    void clear() {
      Index = 0;
    
      unsigned char i;
      for(i=0;i<PID_HISTORY_SIZE;i++) {
        HistoryValues[i] = 0;
      }      
    }
    
    int step(int mValue) {
      unsigned char i;
      int buf = 0;
      int result = 0;
      const int mult = 10;
    
      Index++;
      if (Index == PID_HISTORY_SIZE) Index = 0;
    
      HistoryValues[Index] = mValue;
    
      result = ( mValue - Setting );
    
      for(i=0;i<PID_HISTORY_SIZE;i++)
        {
          buf += HistoryValues[i] - Setting;
        }
      result += buf / t_Integ;
      buf = 0;
    
      unsigned char j_1 = Index + 1;
      if ( j_1 == PID_HISTORY_SIZE ) j_1 = 0;
    
      unsigned char j = Index;
    
      for(i=0;i<(PID_HISTORY_SIZE - 1);i++) {
    
        buf += HistoryValues[j] - HistoryValues[j_1];
    
        j++;
        if ( j == PID_HISTORY_SIZE ) j = 0;
    
        j_1++;
        if ( j_1 == PID_HISTORY_SIZE ) j_1 = 0;
    
      }
    
      result += buf * t_Diff;
      result *= mult;
      result /= Coeff;
    
      return result;      
    }
};

PID pid;

void setup() {
    
  digitalWrite(motorPressUpPin, LOW);
  pinMode(motorPressUpPin, OUTPUT);

  digitalWrite(motorPressDownPin, LOW);
  pinMode(motorPressDownPin, OUTPUT);

  pinMode(waterValvePin, OUTPUT);
  
  pid.Setting = 0;
  
  digitalWrite(32, HIGH); // For SafeDuino
  LCD.Initialize();
    
  LCD.TextGoTo(0, 0);
       
  pinMode(furnacePin, OUTPUT);  
  
  //timer2
  
  TCCR2A = _BV(WGM01); // CTC
  TCCR2B = _BV(CS22);// | _BV(CS20); // clk/64
  OCR2A = 255;


  TIMSK2 = _BV(OCIE2A);  
}


int outputPower = 20; // выходная мощность в %
volatile float value = 0;
volatile float press_value = 0;

long adc_sum = 0;
long press_sum = 0;
bool repaintDisplayFlag = false;

ISR(TIMER2_COMPA_vect) {
  
  static const int maxCounter = 500;
  static const int adcMaxCounter = maxCounter/2;
  static int counter = 0;
  
  static const int repaintDisplayMaxCounter = 1000;
  static int repaintDisplayCounter = 0;    
    
  switch(outputPower) {
    case 100: digitalWrite(furnacePin, HIGH); break;
    case 0: digitalWrite(furnacePin, LOW); break;
    default:
      digitalWrite(furnacePin, counter < (outputPower*(maxCounter/100)) ? HIGH : LOW);   
  }    
  
  
  ++counter;
  if(counter % adcMaxCounter == 0) {
    value = (adc_sum / adcMaxCounter) * 0.863 - 184.0;
    press_value = (press_sum / adcMaxCounter)*0.2866 - 27.429;
    adc_sum = 0;
    press_sum = 0;
  }
  else {
    adc_sum += analogRead(A0);
    press_sum+= analogRead(A2);
  }
  
  if (pid.Setting <= 0)
    outputPower = 0;  
  
  if (counter == maxCounter) {
    counter = 0;
    if (pid.Setting > 0) {
      outputPower -= pid.step(value);
      outputPower = max(outputPower, 0);
      outputPower = min(outputPower, maxOutputPower);    
    }
  }
  
  repaintDisplayCounter++;
  
  if (repaintDisplayCounter == repaintDisplayMaxCounter) {
    repaintDisplayCounter = 0;
    repaintDisplayFlag = true;
  }
  
}


typedef enum  State_t {
  StateInputPressTime = 1, 
  StateInputPressTemperature = 2,
  StateInputPress = 3,
  
  StateInputPreheatTime = 4, 
  StateInputPreheatTemperature = 5,
  StateInputCoolingTime = 6,
  
  StateWork = 7,
  StatePreheat = 8,
  StateCooling = 9,
  StateWaitStart = 10,
  StateRunning = 11,
  StateInputPower = 12,  
};

int currentState = StateWork;

int tempWorkSettings = 160;
int pressWorkSettings = 60;
int timeWorkSettings = 11*60;

int tempPreheatSettings = 80;
int timePreheatSettings = 0;  
int timeCoolingSettings = 6*60;
int currentWorkingTime = 0;
int currentWorkingTimePressing = 0;
int currentWorkingTimeCooling = 0;

void changeState(int newState) {
  currentState = newState;
  LCD.clearGraphic();
}

void printData() {
  if (!repaintDisplayFlag)
    return;
    
      repaintDisplayFlag = false;
      
    LCD.TextGoTo(0, 3);   
    LCD.writeString("                                      ");
    LCD.TextGoTo(0, 4);   
    LCD.writeString("                                      ");

    
    LCD.TextGoTo(0, 1);
    LCD.writeString("Temp ");
    LCD.TextGoTo(0, 2);
    LCD.writeString("( C )");
    LCD.TextGoTo(0+1, 3);
    LCD.writeString(String(tempWorkSettings).c_str());     
    LCD.TextGoTo(0+1, 4);
    LCD.writeString(String(value, 0).c_str());


    LCD.TextGoTo(6, 1);
    LCD.writeString(" Press ");
    LCD.TextGoTo(6, 2);
    LCD.writeString(" (bar) ");
    LCD.TextGoTo(6+1, 3);
    LCD.writeString(String(pressWorkSettings).c_str());     
    LCD.TextGoTo(6+1, 4);
    LCD.writeString(String(press_value, 0).c_str());


    LCD.TextGoTo(13, 1);
    LCD.writeString(" Press ");
    LCD.TextGoTo(13, 2);
    LCD.writeString(" (sec) ");
    LCD.TextGoTo(13+1, 3);
    LCD.writeString(String(timeWorkSettings / 60).c_str());     
    LCD.writeString(":");     
    LCD.writeString(String(timeWorkSettings % 60).c_str());         
    LCD.TextGoTo(13+1, 4);
    LCD.writeString(String(currentWorkingTimePressing / 60).c_str());
    LCD.writeString(":");    
    LCD.writeString(String(currentWorkingTimePressing % 60).c_str());    


    LCD.TextGoTo(20, 1);
    LCD.writeString(" Power");
    LCD.TextGoTo(20, 2);
    LCD.writeString(" ( % )");  
    LCD.TextGoTo(20+2, 3);
    LCD.writeString(String(maxOutputPower).c_str());         
    LCD.TextGoTo(20+2, 4);
    LCD.writeString(String(outputPower).c_str());     


    LCD.TextGoTo(29, 1);
    LCD.writeString(" Water ");
    LCD.TextGoTo(29, 2);
    LCD.writeString(" (sec)");
    LCD.TextGoTo(29+1, 3);
    LCD.writeString(String(timeCoolingSettings / 60).c_str());     
    LCD.writeString(":");     
    LCD.writeString(String(timeCoolingSettings % 60).c_str());      
    LCD.TextGoTo(29+1, 4);
    LCD.writeString(String(currentWorkingTimeCooling / 60).c_str());
    LCD.writeString(":");    
    LCD.writeString(String(currentWorkingTimeCooling % 60).c_str());   
       
      
    LCD.TextGoTo(0, 5);
    LCD.writeString("                                                        ");      
      
    switch(currentState) {
      case StateInputPressTime: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("PRESS TIME = ");
        LCD.writeString(String(timeWorkSettings / 60).c_str());     
        LCD.writeString(":");     
        LCD.writeString(String(timeWorkSettings % 60).c_str());             
      break;
      
      case StateInputPressTemperature: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("PRESS TEMP = ");
        LCD.writeString(String(tempWorkSettings).c_str());     
        LCD.writeString(", `C");             
      break;   
   
      case StateInputPress: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("PRESS = ");
        LCD.writeString(String(pressWorkSettings).c_str());     
        LCD.writeString(", bar");     
      break;      
      
      case StateInputPreheatTime: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("PREHEAT TIME = ");
        LCD.writeString(String(timePreheatSettings / 60).c_str());     
        LCD.writeString(":");     
        LCD.writeString(String(timePreheatSettings % 60).c_str());             
      break;
      
      case StateInputPreheatTemperature: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("PREHEAT TEMP = ");
        LCD.TextGoTo(13, 5);
        LCD.writeString(String(tempPreheatSettings).c_str());     
        LCD.writeString(", `C");             
      break; 

      case StateInputCoolingTime: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("COOLING TIME = ");
        LCD.writeString(String(timeCoolingSettings / 60).c_str());     
        LCD.writeString(":");     
        LCD.writeString(String(timeCoolingSettings % 60).c_str());             
      break; 
 
      case StateInputPower: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("POWER = ");
        LCD.writeString(String(maxOutputPower).c_str());             
      break; 
      
      case StateWaitStart: 
        LCD.TextGoTo(0, 5);
        LCD.writeString("PRESS ON BUTTON TO START OR MENU TO CANCEL");
      break;        
    };       


    LCD.TextGoTo(16, 7);
    LCD.writeString("WATER ");
    if (digitalRead(waterValvePin))
      LCD.writeString("ON ");
    else
      LCD.writeString("OFF");
    
/*     
      if (digitalRead(isClosedPin))
        LCD.writeString("1");  
      else
        LCD.writeString("0");  
        */
}

int onKeyboard = 0;
int prevOnKeyboard = 0;

void inStateWork() {
  
  currentWorkingTimePressing = 0;
  currentWorkingTimeCooling = 0;

  if (onKeyboard & (downKeyMask | upKeyMask)) {
    if (onKeyboard & downKeyMask) 
      analogWrite(motorPressDownPin, 240);    
      
    if (onKeyboard & upKeyMask) 
      analogWrite(motorPressUpPin, 240);    
      
  } else {
    analogWrite(motorPressUpPin, 0);
    analogWrite(motorPressDownPin, 0);    
  }

  if ((onKeyboard & pin3KeyMask) && ((prevOnKeyboard & pin3KeyMask) == 0)) {            
    digitalWrite(waterValvePin, !digitalRead(waterValvePin));
  }


  printData();  
  
  if ((onKeyboard & menuKeyMask) && ((prevOnKeyboard & menuKeyMask) == 0)) {            
    digitalWrite(waterValvePin, LOW);
    changeState(StateInputPreheatTime);
    return; 
  }
    
  if(digitalRead(isClosedPin) == LOW) {
    if ((onKeyboard & onKeyMask) && ((prevOnKeyboard & onKeyMask) == 0)) {   
      digitalWrite(waterValvePin, LOW);      
      changeState(StateRunning);
      return; 
    }    
  }
}


void inStateRunning() {
  
  int press_settings = press_value;
  static int runningStep = 0;
  static unsigned long startingTime = 0;
  
  unsigned long time = millis();
  static bool motorFlag = false;
  
  if (runningStep == 0) { // инициализация
    startingTime == millis();
    //currentWorkingTime = 
    pid.Setting = tempPreheatSettings;
    runningStep = 1;
    motorFlag = false;
    return;
  }
 
  if (runningStep == 1) { // предварительный нагрев

    if (press_settings > pressWorkSettings) {
      analogWrite(motorPressUpPin, 0);
      motorFlag = false;
    }
  
    if ((press_settings < (pressWorkSettings - 5)) && !motorFlag) { 
      if (pressWorkSettings < 61)
        analogWrite(motorPressUpPin, 140);
      else {
        if(pressWorkSettings < 160)
          analogWrite(motorPressUpPin, 220);
        else
          analogWrite(motorPressUpPin, 240);
      }
      motorFlag = true;
    }
  
    currentWorkingTime = (time - startingTime)/1000;
    if ( currentWorkingTime > timePreheatSettings) {
      startingTime = time;
      runningStep = 2;
      pid.Setting = tempWorkSettings;
      analogWrite(motorPressUpPin, 0);
      motorFlag = false;
      return;
    }
  }
  
  if (runningStep == 2) { // работа
        
    if (press_settings > pressWorkSettings) {
      analogWrite(motorPressUpPin, 0);
      motorFlag = false;
    }
  
    if ((press_settings < (pressWorkSettings - 5)) && !motorFlag) { 
      if (pressWorkSettings < 61)
        analogWrite(motorPressUpPin, 140);
      else {
        if(pressWorkSettings < 160)
          analogWrite(motorPressUpPin, 200);
        else
          analogWrite(motorPressUpPin, 240);
      }
        
      motorFlag = true;
    }
    
    currentWorkingTime = (time - startingTime)/1000;
    
    currentWorkingTimePressing = currentWorkingTime;
    
    if ( currentWorkingTime > timeWorkSettings) {
      analogWrite(motorPressUpPin, 0);
      pid.Setting = 0;
      startingTime = time;
      runningStep = 3;
      return;
    }    
    //pressWorkSettings
  }
  
  if (runningStep == 3) { // охлаждение
    digitalWrite(waterValvePin, HIGH);
    currentWorkingTime = (time - startingTime)/1000;
    currentWorkingTimeCooling = currentWorkingTime;    
    if ( currentWorkingTime > timeCoolingSettings) {
      startingTime = time;
      runningStep = 0;
      digitalWrite(waterValvePin, LOW);
      pid.Setting = 0;
      analogWrite(motorPressUpPin, 0);  
      runningStep = 0;
      changeState(StateWork);      
      return;
    }      
  }
  
  bool rep = repaintDisplayFlag;
  printData(); 
  if(rep) {
    LCD.TextGoTo(0, 5);
    switch(runningStep) {
      case 1: LCD.writeString("PREHEATING "); break;
      case 2: LCD.writeString("HEATING   "); break;
      case 3: LCD.writeString("COOLING   "); break;
      default: 
        LCD.writeString("RUNNING   ");
    }
        
  }
  
  if ((onKeyboard & offKeyMask) && ((prevOnKeyboard & offKeyMask) == 0)) {  
    pid.Setting = 0;
    analogWrite(motorPressUpPin, 0);  
    runningStep = 0;
    changeState(StateWork);
    return; 
  }   
}


void inputValueState(int &value, int minV, int maxV, int stepV, int nextState) {
     
  if ((onKeyboard & downKeyMask) && ((prevOnKeyboard & downKeyMask) == 0)) {
    value -= stepV; 
  }
        
  if ((onKeyboard & upKeyMask) && ((prevOnKeyboard & upKeyMask) == 0)) {
    value += stepV;
  }

  value = max(value, minV);
  value = min(value, maxV);

  printData();
  
  if ((onKeyboard & menuKeyMask) && ((prevOnKeyboard & menuKeyMask) == 0)) {            
    changeState(nextState);
    return; 
  }
  
}


void loop() {
  
  static int onKeyboardBuf = 0; // для антидребезга
  static int onKeyboardBufCounter = 0; // для антидребезга
  static const int onKeyboardBufMaxCounter = 100; // для антидребезга
  
  for(int i=0; i<sizeof(keyboardPins); ++i) {
    onKeyboardBuf |= digitalRead(keyboardPins[i]) ? (1 << i) : 0;
  }  
   
  ++onKeyboardBufCounter;
  if(onKeyboardBufCounter == onKeyboardBufMaxCounter) {    
    onKeyboard = onKeyboardBuf;
    onKeyboardBuf = 0;
    onKeyboardBufCounter = 0;
  }
  
  
  switch(currentState) {
    case StateWork: inStateWork(); break;
    // inputValueState(int &value, int minV, int maxV, int stepV, int nextState)
    case StateInputPreheatTime: inputValueState(timePreheatSettings, 0, 60*500, 30, StateInputPreheatTemperature); break;
    case StateInputPreheatTemperature: inputValueState(tempPreheatSettings, 0, 100, 10, StateInputPressTime); break;    
    case StateInputPressTime: inputValueState(timeWorkSettings, 0, 60*500, 30, StateInputPressTemperature); break;
    case StateInputPressTemperature: inputValueState(tempWorkSettings, 150, 180, 10, StateInputPress); break;
    case StateInputPress: inputValueState(pressWorkSettings, 5, 230, 10, StateInputCoolingTime); break;
    case StateInputCoolingTime: inputValueState(timeCoolingSettings, 0, 60*500, 30, StateInputPower); break;
    case StateInputPower: inputValueState(maxOutputPower, 5, 35, 5, StateWork); break;
    case StateRunning: inStateRunning(); break;
  };


  prevOnKeyboard = onKeyboard;
  
}



