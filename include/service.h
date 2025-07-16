#ifndef SERVICE_H
#define SERVICE_H   

void serviceTick();  
void serviceIN();    
void serviceOUT();   

#if (START_MENU == 1)
void drawStartMenu(int8_t pos);
void startupService();
void startMenu();
#endif

#endif