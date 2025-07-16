#ifndef A0_DATA_H
#define A0_DATA_H

#include <Arduino.h>
#include <stdint.h>

#pragma once

#define START_MENU 0          
#define USE_PLOTS 0           
#define USE_DAWN 0            
#define USE_DRIVE 0           


#define SERVO1_RELAY 1        
#define SERVO2_RELAY 1        
#define SMOOTH_SERVO 0        
#define SERVO_MIN_PULSE 500   
#define SERVO_MAX_PULSE 2500  


#define USE_PID 0             
#define USE_PID_RELAY 0       
#define SCHEDULE_NUM 0        
#define PID_INT_MIN -3000     
#define PID_INT_MAX 3000      
#define PID_AUTOTUNE 0        
#define DEBUG_PID 0           




#define SHOW_INTEGRAL 1       


#define ENCODER_TYPE 1        
#define ENC_REVERSE 1         
#define CONTROL_TYPE 1            
#define FAST_TURN 1           
#define FAST_TURN_STEP 10     


#define DRIVER_LEVEL 1        
#define DRIVE_TOUT 10         
#define PWM_RELAY_HZ 1        
#define SETT_TIMEOUT 100      
#define WIRE_OVERCLOCK 1      
#define LCD_ADDR 0x27         
#define BME_ADDR 0x76         

#define WDT_ENABLE 0          






#define USE_CO2 0           
#define CO2_PIN 1           
#define CO2_MAX 5000        
#define CO2_CALIB 0         



#define USE_BME 0           



#define USE_HTU21D 0        

#define USE_BMP280 1

#define USE_AHT20 1



#define DALLAS_SENS1 0      
#define DALLAS_AMOUNT 1     


#define DALLAS_MODE 0       



#define DALLAS_DISP 0       




#if (DALLAS_AMOUNT > 1)
const uint8_t dsAddress[][8] = {
  {0x28, 0xFF, 0x42, 0x5A, 0x51, 0x17, 0x4, 0xD2}, 
  {0x28, 0xFF, 0x53, 0xE5, 0x50, 0x17, 0x4, 0xC3}, 
  {0x28, 0xFF, 0x99, 0x80, 0x50, 0x17, 0x4, 0x4D}, 
  
};
#endif



// #define DHT_SENS2 0         
// #define DHT_TYPE DHT22      



#define THERM1 0            
#define THERM2 0            
#define THERM3 0            
#define THERM4 0            

#define BETA_COEF1 3435     
#define BETA_COEF2 3435     
#define BETA_COEF3 3435     





extern const uint8_t daysMonth[];
extern const char *channelNames[];
extern const char *modeSettingsNames[];
extern const char *modeNames[];
extern const char *relayNames[];
extern const char *modeNames[];
extern const char *directionNames[];
extern const char *settingsNames[];
extern const char *settingsPageNames[];
extern const char *sensorNames[];
extern const char *sensorUnits[];

#if (USE_DAWN == 1)
extern const char *dawnNames[];
#endif

#if (USE_PID == 1)
extern const char *pidNames[];
#endif

#if (USE_PLOTS == 1 || USE_PID == 1 || USE_DAWN == 1)

extern const char row8[] PROGMEM;
extern const char row7[] PROGMEM;
extern const char row6[] PROGMEM;
extern const char row5[] PROGMEM;
extern const char row4[] PROGMEM;
extern const char row3[] PROGMEM;
extern const char row2[] PROGMEM;
extern const char row1[] PROGMEM;
#endif

#if (START_MENU == 1)
extern bool startService = false;
#endif

extern int8_t currentChannel; 
extern boolean backlState;
extern byte curPWMchannel; 
extern byte curSetMode;   

#include "a1_data.h"

#endif