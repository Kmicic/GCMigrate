#ifndef AUTOMATICS_H
#define AUTOMATICS_H
#include <Arduino.h>

void timersTick();   // co sekundę
void checkDawn(byte curChannel); // podświetlenie LCD
byte checkHysteresis(byte channel); // проверка гистерезиса 
void getDay(); // numer dnia  
boolean checkDay(byte curChannel); // dzień tygodnia

#if (SCHEDULE_NUM > 0)
void checkShedule(); // проверка расписания
#endif

#endif