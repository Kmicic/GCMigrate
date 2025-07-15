#include "a0_data.h"
#include "a4_func.h"
#include "sensors.h"
#include "service.h"
#include "automatics.h"
#include "redrawScreen.h"
#include "pid.h"



void controlTick();



void loop() {
  customLoop();       
  checkPID();         
  backlTick();        
  debTick();          
  controlTick();      
  readAllSensors();   

#if (USE_CO2 == 1)
  CO2tick();
#endif

#if (USE_PLOTS == 1)
  plotTick();         
#endif

#if (USE_DRIVE == 1)
  driveTick();        
#endif

#if (USE_PID_RELAY == 1)  
  tickPWM(0, flag0, tmr0);
  tickPWM(1, flag1, tmr1);
#endif

#if (SERVO1_RELAY == 0 && SMOOTH_SERVO == 1)
  servo1.tick();      
#endif
#if (SERVO2_RELAY == 0 && SMOOTH_SERVO == 1)
  servo2.tick();      
#endif

  if (currentChannel == -3) {         
    serviceTick();
  } else {  
    if (millis() - commonTimer > 1000) {
      commonTimer += 1000;
      timersTick();
#if (USE_CO2 == 0)
      if (currentChannel == -1) {
        if (debugPage == 0) redrawDebug();
#if (PID_AUTOTUNE == 1)
        else if (debugPage == 1) redrawTuner();
#endif
        else redrawPlot();
      }
#endif
    }
#if (USE_CO2 == 1)
    static uint32_t comTimer2;
    if (millis() - comTimer2 > 1500) {
      comTimer2 = millis();
      CO2_rst = true;
      if (currentChannel == -1) {
        if (debugPage == 0) redrawDebug();
#if (PID_AUTOTUNE == 1)
        else if (debugPage == 1) redrawTuner();
#endif
        else redrawPlot();
      }
    }
#endif
  }

#if (WDT_ENABLE == 1)
  wdt_reset();        
#endif
}
