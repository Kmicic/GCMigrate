#include "a0_data.h"
#include "arrowcontrol.h"
#include "redrawScreen.h"
#include "a4_func.h"
#include "custom.h"

void setup() {
Serial.begin(9600);








#if (DEBUG_PID > 0)
#if (PID_AUTOTUNE == 1)
  serial.println("input , min , max");
#else
#if (SHOW_INTEGRAL == 1)
  serial.println("set , input , integral/3 , out");
#else
  serial.println("set , input , out");
#endif
#endif
#endif

  boolean startupPress = false;
  initHardware();
  
#if (START_MENU == 0)
  
  if (!digitalRead(SW)) {
    startupPress = true;
    lcd.setCursor(0, 0);
    lcd.print(F("Reset settings OK"));

  }
  while (!digitalRead(SW));
#else
  
  EEPROM.get(EEPR_SETTINGS, settings);
  if (!digitalRead(SW)) {
    drawStartMenu(0);
    while (!digitalRead(SW));
    startMenu();
  }

#endif

  
  if (EEPROM.read(EEPR_KEY_ADDR) != EEPR_KEY || startupPress) {
    clearEEPROM();  
  }
  EEPROM.get(EEPR_SETTINGS, settings);     
  applySettings();  

  
  currentChannel = -1;  
  currentLine = 4;
  drawArrow();
  redrawScreen();
  disableABC();
  customSetup();        
}

ISR(INT0_vect) {        
  enc.tick(controlState);
}

ISR(INT1_vect) {
  enc.tick(controlState);
}
