#ifndef AUTOMATICS_H
#define AUTOMATICS_H
#include <Arduino.h>

void timersTick();   
void checkDawn(byte curChannel); 
byte checkHysteresis(byte channel); 
void getDay(); 
boolean checkDay(byte curChannel); 

#if (SCHEDULE_NUM > 0)
void checkShedule(); 
#endif

#endif