#define C4NODEMCU_GPIO

#include <ESP8266WiFi.h>
#include <PCF8575.h>
#include <Servo.h>
#include "C4-NodeMCU.h"

void gpioSet(int numRelay, int newState) {
    if (numRelay < 0 || numRelay >= MAX_GPIOS) {
        serprln("gpioSet: index out of range: " + String(numRelay));
        return;
    }
    if (config_gpio_use_expander=="0")
        digitalWrite(config_gpio[numRelay].toInt(), newState);
    else if (config_gpio_use_expander=="1")
        pcf8575.digitalWrite(config_gpio[numRelay].toInt(), newState);
    else if (config_gpio_use_expander=="2")
        pcf8575.digitalWrite(config_gpio[numRelay].toInt(), newState);
    else {
        serpr("gpioSet: NOT IMPLEMENTED ");
        serpr(numRelay);
        serpr("state ");
        serprln(newState);
    }
}

boolean gpioState(int numRelay) {
  if (config_gpio_use_expander=="0")
      return digitalRead(config_gpio[numRelay].toInt());
  if (config_gpio_use_expander=="1")
    return pcf8575.digitalRead(config_gpio[numRelay].toInt());
  if (config_gpio_use_expander=="2")
    return pcf8575.digitalRead(config_gpio[numRelay].toInt());

  return false;
}

// Get the status of each individual relay
String relayState(int numRelay) {
  if(RELAY_NO) {
    if(gpioState(numRelay-1))
      return "";
    else
      return "checked";
  }
  else {
    if(gpioState(numRelay-1))
      return "checked";
    else
      return "";
  }
  return "";
}

// Move actuator to predefined angle
void angle(int m, int a) {
    //serpr("ANGLE: motor ");
    //serpr(m);
    //serpr(", angle ");
    //serprln(a);
    motor[m].write(a);
    delay(5*abs(oapos[m]-a)+50);
    oapos[m]=a;
}

// Check if GPIO is allowed to be controlled
bool is_valid_gpio(String which) {
  if (config_gpio_use_expander=="1" && which.toInt()<8)
      return true;
  if (config_gpio_use_expander=="2" && which.toInt()<16)
      return true;
  for (int j=0; j<10; j++) {
    if (which==valid_gpio[j]) {
      return true;
    }
  }
  return false;
}
