#ifndef REDRAWSCREEN_H
#define REDRAWSCREEN_H  
#include <Arduino.h>
#include "a0_data.h"

void printDash();
void printSpace();
void printOneSpace();
void printColon();
void spaceColon();
void clearLine();
void clearLine2(byte row);
void printOff();
void printOn();
void drawPlot(int *plot_array);
void redrawPlot();
void redrawDebug();

#if (PID_AUTOTUNE == 1)
void redrawTuner();
#endif

#if (SCHEDULE_NUM > 0)
void printDate(int day365);
void redrawSchedule();
#endif

void redrawChannels();
void redrawSettings();
void printName();
void redrawService();
void redrawScreen();
void redrawDebug();
void redrawMainSettings();

#endif