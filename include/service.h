#ifndef SERVICE_H
#define SERVICE_H   
#include "a0_data.h"

void serviceTick();  // сервисный таймер
void serviceIN();    // выполняем при входе в сервис    
void serviceOUT();   // выполняем при выходе из сервиса

#if (START_MENU == 1)
void drawStartMenu(int8_t pos);
void startupService();
void startMenu();
#endif

#endif