#include "a0_data.h"

#if (PID_AUTOTUNE == 1)
// relay
#define P_PI 0.4f
#define I_PI 0.48f
#define P_PID 0.6f
#define I_PID 1.2f
#define D_PID 0.075f

#define BUF_SIZE 6
uint32_t startTime = millis();
float lastInput = 0;
float lastInputWindow = 0;
float steadyValue = 0;
byte workstate = 0;
bool trigger = true;
bool changeDir = false;
bool signalDirection = false;
static float buf[BUF_SIZE];
int32_t prevOscTime = 0;
float thisInput = 0;

void tunePID() {
  if (tunerSettings.restart) {
    tunerSettings.restart = 0;
    tuner.cycle = 0;
    tuner.value = tunerSettings.steady;
    tuner.P = 0;
    tuner.I = 0;
    tuner.D = 0;
    startTime = millis();
    workstate = 0;
    trigger = true;
    changeDir = false;
    signalDirection = false;
    tuner.status = tunerSettings.tuner;
  }
  static uint32_t tmr;
  if (tunerSettings.tuner &&
      !tunerSettings.manual &&
      tunerSettings.channel > 0 &&
      millis() - tmr >= tunerSettings.period * 1000L) {
    tmr += tunerSettings.period * 1000L;
    tuner.input = sensorVals[tunerSettings.sensor];
    //thisInput += (tuner.input - thisInput) * 0.5;
    thisInput = tuner.input;
    if (workstate != 3) {
      tuner.min = tuner.input;
      tuner.max = tuner.input;
    }

    switch (workstate) {
      case 0:   
        if (millis() - startTime >= tunerSettings.delay * 1000L) {    
          startTime = millis();
          if (abs(thisInput - lastInputWindow) < tunerSettings.window) {    
            startTime = millis();
            workstate = 1;
            tuner.status = 1;
          }
          lastInputWindow = thisInput;
        }
        break;
      case 1:   
        if (millis() - startTime >= tunerSettings.delay * 1000L) { 
          startTime = millis();
          steadyValue = thisInput;                
          tuner.max = steadyValue;
          tuner.min = steadyValue;
          tuner.value = (tunerSettings.steady - tunerSettings.step);    
          workstate = 2;
          tuner.status = 2;
        }
        break;
      case 2:   
        if (millis() - startTime >= tunerSettings.kickTime * 1000L) {
          startTime = millis();
          tuner.value = (tunerSettings.steady + tunerSettings.step);    
          workstate = 3;
          tuner.status = 3;
        }
        break;
      case 3:   
        
        float aver = 0;
        for (byte i = 0; i < BUF_SIZE - 1; i++) {
          aver += buf[i];
        }
        aver /= (BUF_SIZE - 1);
        signalDirection = (aver < buf[BUF_SIZE - 1]) ? true : false;

        if (!signalDirection) {             
          if (changeDir && !trigger) {      
            changeDir = false;
            tuner.max = buf[BUF_SIZE - 2];  
          }
          if (thisInput < steadyValue) {    
            if (!trigger) {
              trigger = 1;
              tuner.value = (tunerSettings.steady + tunerSettings.step);   
              tuner.status = 3;
            }
          }
        } else if (signalDirection) {       
          if (!changeDir && trigger) {      
            changeDir = true;
            tuner.min = buf[BUF_SIZE - 2];  
          }
          if (thisInput > steadyValue) {    
            if (trigger) {
              trigger = 0;
              tuner.value = (tunerSettings.steady - tunerSettings.step);   
              tuner.status = 4;

              
              tuner.cycle = millis() - startTime;
              startTime = millis();
              float Ku = 4.0f * tunerSettings.step / (tuner.max - tuner.min) / PI;
              float Pu = tuner.cycle / 1000.0f;

              if (tunerSettings.result) {
                tuner.P = P_PI * Ku;
                tuner.I = I_PI * Ku / Pu;
              } else {
                tuner.P = P_PID * Ku;
                tuner.I = I_PID * Ku / Pu;
                tuner.D = D_PID * Ku * Pu;
              }
            }
          }
        }
        break;
    }
    for (byte i = 0; i < BUF_SIZE - 1; i++) {
      buf[i] = buf[i + 1];
    }
    buf[BUF_SIZE - 1] = thisInput;

    lastInput = thisInput;
#if (DEBUG_PID > 0)
    uart.print(thisInput); uart.print(' ');
    //uart.print(tuner.value); uart.print(' ');
    uart.print(tuner.min); uart.print(' ');
    uart.println(tuner.max);
#endif
  }
}
#endif
