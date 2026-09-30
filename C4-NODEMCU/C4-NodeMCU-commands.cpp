#define C4NODEMCU_COMMANDS
#include "C4-NodeMCU.h"

// Aeotec light
void aeotec_light_add(int m) {
  serpr("AEOTEC_LIGHT_ADD: ");
  serprln(m);
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (digitalRead(config_gpio[m].toInt())) {
      gpioSet(m, 0);
      delay(2000);
    }
    for (int i=0; i<3; i++) {
      gpioSet(m, 1);
      delay(1000);
      gpioSet(m, 0);
      delay(1000);
    }
  }
}

// Aeotec light
void aeotec_light_remove(int m) {
  serpr("AEOTEC_LIGHT_REMOVE: ");
  serprln(m);
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (digitalRead(config_gpio[m].toInt())) {
      gpioSet(m, 0);
      delay(1500);
    }
    for (int i=0; i<3; i++) {
      // Turn light off
      gpioSet(m, 1);
      delay(1200);
      // Turn light on
      gpioSet(m, 0);
      delay(800);
    }
  }
  delay(5000);
  gpioSet(m, 1);
}

// Function to perform inclusion of Fibaro and Vesta Outlet Plug modules
void fibaro_add(int m) {
  serpr("FIBARO_ADD: ");
  serprln(m);
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (!digitalRead(config_gpio[m].toInt())) {
      gpioSet(m, 1);
      delay(1000);
    }
    for (int i=0; i<3; i++) {
      gpioSet(m, 0);
      delay(200);
      gpioSet(m, 1);
      delay(200);
    }
  } else if (config_device_type == DEVICE_TYPE_ACTUATOR) {
    if (oapos[m] < max_angle[m].toInt())
      angle(m, max_angle[m].toInt());
    for (int i=0; i<3; i++) {
      angle(m, min_angle[m].toInt());
      angle(m, min_angle[m].toInt()+15);
    }
    angle(m, max_angle[m].toInt());
  }
}

// Function to perform inclusion of Fibaro and Vesta Outlet Plug modules
void vesta162(int m) {
  serpr("VESTA162_ADDRM: ");
  serprln(m);
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (!digitalRead(config_gpio[m].toInt())) {
      gpioSet(m, 1);
      delay(500);
    }
    for (int i=0; i<3; i++) {
      gpioSet(m, 0);
      delay(100);
      gpioSet(m, 1);
      delay(100);
    }
  }
}

// Function to fake flood on Fibaro flood sensor
void fibaro_flood_trigger(int m) {
  serpr("FIBARO_FLOOD: ");
  serprln(m);
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (!digitalRead(config_gpio[m].toInt())) {
      gpioSet(m, 1);
      delay(1000);
    }
    for (int i=0; i<3; i++) {
      gpioSet(m, 0);
      delay(100);
      gpioSet(m, 1);
      delay(50);
    }
    gpioSet(m, 0);
  }
}

// Toggle outlet
void toggle(int m) {
  //serpr("TOGGLE: ");
  //serprln(m);
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (!gpioState(m)) {
      gpioSet(m, 1);
      delay(1000);
    }
    gpioSet(m, 0);
    delay(200);
    gpioSet(m, 1);
  } else if (config_device_type == DEVICE_TYPE_ACTUATOR) {
    if (oapos[m] != max_angle[m].toInt()) {
      angle(m, max_angle[m].toInt());
      delay(500);
    }

    angle(m, min_angle[m].toInt());
    delay(200);
    angle(m, min_angle[m].toInt()+15);
    delay(100);

    angle(m, max_angle[m].toInt());
  }
}

void button_toggle(int m, int times) {
  if (m < 0 || m >= MAX_GPIOS) {
    serprln("button_toggle: index out of range: " + String(m));
    return;
  }
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (!digitalRead(config_gpio[m].toInt())) {
      gpioSet(m, 1);
      delay(800);
    }
    for (int i=0; i<times; i++) {
      gpioSet(m, 0);
      delay(150);
      gpioSet(m, 1);
      delay(150);
    }
  }
}

// Function to perform inclusion on Control4 Zigbee Puck devices
void control4_puck_leave_mesh(int m) {
  serpr("CONTROL4_Puck_Identify: ");
  serprln(m);
  button_toggle(m, 13);
}

// Function to perform inclusion on Control4 Zigbee devices
void control4_identify(int m) {
  serpr("CONTROL4_Identify: ");
  serprln(m);
  button_toggle(m, 4);
}

// Function to perform factory default on Control4 Zigbee devices
void control4_channel(int m) {
  serpr("CONTROL4_Channel: ");
  serprln(m);
  button_toggle(m, 7);
}

// Function to perform factory default on Control4 Zigbee devices
void control4_reboot(int m) {
  serpr("CONTROL4_Reboot: ");
  serprln(m);
  button_toggle(m, 15);
}

// Function to perform factory default on Control4 Zigbee devices
void control4_factory_reset(int m) {
  serpr("CONTROL4_Reset: ");
  serprln(m);
  button_toggle(m,    9);
  button_toggle(m+1,  4);
  button_toggle(m,    9);
}

// Function to perform factory default on Control4 Zigbee devices
void control4_leave_mesh(int m) {
  serpr("CONTROL4_LEAVE_MESH: ");
  serprln(m);
  button_toggle(m,    13);
  button_toggle(m+1,  4);
  button_toggle(m,    13);
}

// Function to perform inclusion on IKEA Zigbee devices
void ikea_tretakt_identify(int m) {
  serpr("IKEA_TRETAKT_Identify: "); serprln(m);
  button_toggle(m+1, 1);
}

// Function to perform factory default on IKEA Zigbee devices
void ikea_tretakt_factory_reset(int m) {
  serpr("IKEA_TRETAKT_Reset: "); serprln(m);
  button_toggle(m+1, 5);
}

// Function to perform Toggle command on IKEA Zigbee devices
void ikea_tretakt_toggle(int m) {
  serpr("IKEA_TRETAKT_Toggle: "); serprln(m);
  button_toggle(m,   1);
}
