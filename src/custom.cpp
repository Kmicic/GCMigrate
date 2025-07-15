#include "a0_data.h"

/*
  На данной вкладке можно вручную дописать контроллеру дополнительные возможности.
  - Можно использовать пины D1, D4, D5, D6, D7, D8, D9 как цифровые ВЫХОДЫ и ВХОДЫ
  - Соответствующие каналы нужно отключить из меню дисплея (-> Off), иначе работать не будет
  - В customSetup нужно инициализировать выход как INPUT/OUTPUT
*/

boolean chkTimer (unsigned long &startTime, unsigned long loopTime) 
{ if (millis()-startTime >= loopTime)  {      
        startTime = millis();          
        return true;  }                
  else return false;  
}

void customSetup() {
timerMillis[0] = millis();
timerMillis[1] = millis();
  // инициализация для собственных алгоритмов работы

}

void customLoop() {

  // вызывается вместе с основным loop
#if (DEBUG_ENABLE==1)  
  if (chkTimer(timerMillis[1], 5000)) {
    Serial.print("currentChannel:"); Serial.println(currentChannel);    
    Serial.print("Sensor        :"); Serial.println(sensorNames[setChannel.sensor]);
    Serial.print("Relay Type    :"); Serial.println(setChannel.relayType);
    Serial.print("threnshold    :"); Serial.println(setChannel.threshold);
    Serial.print("thrensholdMax :"); Serial.println(setChannel.thresholdMax);
    Serial.print("Value         :"); Serial.println(sensorVals[setChannel.sensor]);  
  timerMillis[1]=millis();  
  }

 
#endif
}
