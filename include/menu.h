#ifndef MENU_H
#define MENU_H
#include <Arduino.h>

#include "a0_data.h"
#include "a4_func.h"

void incr(boolean* val, int incr);    
void incr(int* val, int incr, int limit);
void incr(uint32_t* val, int incr, uint32_t limit);
void incr(int8_t* val, int incr, int limit);
void incr(float* val, float incr, float limit);
void incr(byte* val, int incr, int limit);
void incrInt(int* val, int incr, int limit);
void drawDebug();
void changeChannel(int dir);
#if (SCHEDULE_NUM > 0)
void scheduleSett(int dir);
#endif
void serviceSett(int dir);
void settingsSett(int dir);
void setDMY(byte set, int dir);
void channelSett(int dir);
void chSettingsSett(int dir);
void controlTick();
void recalculateTime();
void correctTime();
void s_to_hms();
void hms_to_s();


#endif // MENU_H