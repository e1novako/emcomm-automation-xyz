/**
 * \brief       Firmware for C4-NODEMCU automation device
 * \details     Provides a way to automaticaly add/remove/devices in Z-Wave and Zigbee mesh using relays and actuators
 *
 * \copyright   Copyright 2023 Snap One, LLC. All Rights Reserved.
 *
 * \par v3.2.1 - Security/stability fixes
 *  - Added HTTP Basic Auth (admin / current Wi-Fi password) to /apply_config, /restart_device,
 *    /device_defaults, and /doUpdate (OTA firmware upload).
 *  - Bounds-checked relay/actuator index parameters on /update and /actuator before array access.
 *  - Clamped configured output count ("outputs") to MAX_GPIOS on both HTTP input and flash load.
 *  - Fixed 1-byte heap overflow and unsafe self-referential sscanf() when parsing MAC address.
 *  - Fixed uptime() string buffer overflow beyond ~100 hours of runtime.
 *  - Added bounds checks in gpioSet()/button_toggle() to prevent out-of-bounds GPIO array access.
 *  - Deferred Wi-Fi reconnect from the disconnect event callback to the main loop.
 *  - Added a runtime debug-output toggle (Settings page), persisted in configuration.
*/

#define C4NODEMCU_MAIN

// Our project includes ...
#include "C4-NodeMCU.h"
#include "C4-NodeMCU-config.h"
#include "C4-NodeMCU-WWW.h"

// ESP8266
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>

// WIFI Events
#include <ESP8266WiFi.h>

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

// Set by the WiFi disconnect event; actual reconnect is deferred to loop()
// to avoid re-entering the ESP8266 WiFi stack from within its own event callback.
volatile bool wifi_reconnect_pending = false;

// Servo support
#include <Servo.h>
// Port expander library
#include <PCF8575.h>

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi sucessfully.");
  Serial.print("IP address:  ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI:        ");
  Serial.println(WiFi.RSSI());}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi, trying to connect...");
  wifi_reconnect_pending = true;
}

// Print data about flasn chip on serial port
void show_flash_size() {
uint32_t realSize = ESP.getFlashChipRealSize();
uint32_t ideSize = ESP.getFlashChipSize();
FlashMode_t ideMode = ESP.getFlashChipMode();

  serprf("\n\nFlash real id:   %08X\n", ESP.getFlashChipId());
  serprf("Flash real size: %u bytes\n\n", realSize);

  serprf("Flash ide  size: %u bytes\n", ideSize);
  serprf("Flash ide speed: %u Hz\n", ESP.getFlashChipSpeed());
  serprf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) {
    serprln("Flash Chip configuration wrong!\n");
  } else {
    serprln("Flash Chip configuration ok.\n");
  }
}

// Configure outputs on device, according to provided parameter
void setup_outputs(String devt) {
  if (config_device_type == DEVICE_TYPE_RELAY) {
    if (config_gpio_use_expander.toInt()==0) {
        // Set all relays to off
        for(int i=1; i<=NUM_OUTPUTS; i++){
          serprln("Checking GPIO #" + String(i) + " - pin " + String(config_gpio[i]));
          //delay(200);
          if (is_valid_gpio(config_gpio[i-1])) {
            //serprln("Configuring GPIO " + String(i) + " - pin " + String(config_gpio[i]));
            //delay(200);
            if (GPIO_CAPABILITY(i-1, GPIO_CAPABILITY_OUTPUT)) {
              pinMode(config_gpio[i-1].toInt(), OUTPUT);
              if(RELAY_NO){
                gpioSet(i-1, HIGH);
              }
              else{
                gpioSet(i-1, LOW);
              }
            } else {
              pinMode(config_gpio[i-1].toInt(), INPUT);
            }
          } else {
            serprln("Invalid GPIO " + String(i) + " - pin " + String(config_gpio[i]));        
          }
        }
    } else if (config_gpio_use_expander.toInt()>0 && config_gpio_use_expander.toInt()<3) {
        // Set pinMode to OUTPUT
        for(int i=0;i<NUM_OUTPUTS;i++) {
          pcf8575.pinMode(i, OUTPUT);
        }
        
        // Set lower speed with idea to gain stability
        Wire.setClock(50000);

        pcf8575.begin();
        for(int i=0;i<NUM_OUTPUTS;i++) {
          pcf8575.digitalWrite(i, HIGH);
        }
    }
  } else if (config_device_type == DEVICE_TYPE_ACTUATOR) { 
    serprln("Move motor to default position...");
    for (int i=0; i<NUM_OUTPUTS && i<MAX_GPIOS; i++) {
      if (is_valid_gpio(config_gpio[i])) {
        serpr("---> Move motor ");
        serpr(i);
        serprln(" to default position");    
        pinMode(config_gpio[i-1].toInt(), OUTPUT);
        motor[i].attach(config_gpio[i].toInt());
        motor[i].write(max_angle[i].toInt()+50);
        oapos[i]=max_angle[i].toInt();
      }
    }
    delay(300);
    for (int i=0; i<NUM_OUTPUTS && i<MAX_GPIOS; i++) {
      if (is_valid_gpio(config_gpio[i])) {
        serpr("---> Move motor ");
        serpr(i);
        serprln(" to default position");    
        motor[i].attach(config_gpio[i].toInt());
        motor[i].write(max_angle[i].toInt());
        oapos[i]=max_angle[i].toInt();
      } else {
        serprln("Invalid gpio " + String(i) + " - pin " + String(config_gpio[i]));        
      }
    }
  } else {
    serpr("Unknown DEVICE_TYPE ");
    serprln(config_device_type);
  }
}

void WiFiOn() {
	wifi_fpm_do_wakeup();
	wifi_fpm_close();

	//Serial.println("Reconnecting");
	wifi_set_opmode(STATION_MODE);
	wifi_station_connect();
}


void WiFiOff() {
	wifi_station_disconnect();
	wifi_set_opmode(NULL_MODE);
	wifi_set_sleep_type(MODEM_SLEEP_T);
	wifi_fpm_open();
	wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
}


// Default setup function
void setup(void) {
  // Disble RF in order to preserve power on start
  WiFi.setOutputPower(0);
  //WiFiOff();
  //ESP.deepSleep(1e6 * 9, WAKE_RF_DISABLED);
  //ESP.deepSleep(1e6 * 7, WAKE_RF_DEFAULT);

  // Serial port for debugging purposes
  Serial.begin(115200);
  delay(200);

  // Print flash size and configuration on serial port
  show_flash_size();

  // Get default MAC address
  wifi_get_macaddr(STATION_IF, MAC);

  // Read the configuration and populate variables...
  read_config();

  // Configure outputs and set default values
  setup_outputs(config_device_type);

  serprln("Waiting for other peripherals to stabilize...");

  // WakeUP WIFI
  //WiFiOn();

  delay(8000);
  WiFi.setOutputPower(rf_power.toInt());

  // Prepare WIFI
  WiFi.disconnect(true);

  //
  //
  //
  WiFi.mode(WIFI_STA);

  wifi_set_macaddr(STATION_IF, &MAC[0]);


  //Register event handlers
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  // Prepare for WIFI connection ...
  WiFi.mode(WIFI_STA);
  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);

  serprln("Setting MAC address to " + macToString(MAC));
  if (wifi_set_macaddr(STATION_IF, &MAC[0]) == 0) {
    serprln("MAC address changed to " + macToString(MAC));
  } else {
    serprln("Unable to change MAC address.");
  }

  // Get Current Hostname
  serpr("\n\nDefault hostname: ");
  serprln(WiFi.hostname().c_str());

  // Append newHostname to default hostname
  newHostname = config_device_type;
  newHostname += WiFi.hostname();
  if (newHostname != WiFi.hostname()) {
    WiFi.hostname(newHostname.c_str());
    serpr("New Hostname: ");
    serprln(WiFi.hostname().c_str());
    //delay(1000);
    //LittleFS.end();
    //ESP.restart();
  }

  // Connect to WIFI ... Use ssid and password to connect
  serpr("\nConnecting " + macToString(MAC) + " to WiFi - " + ssid + " .");
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  wifi_set_macaddr(SOFTAP_IF, &MAC[0]);
  while (WiFi.status() != WL_CONNECTED) {
    serpr(".");
    delay(1000);
    //serprln("Connecting to WiFi..");
  }
  serprln("\nConnecting " + macToString(MAC) + " to WiFi.. connected!");
  previousMillis = millis();

  // Set AutoReconnect !!!
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // Print obtained IP Address
  serpr("Obtained IP address: ");
  serprln(WiFi.localIP());

  // Start all network services (HTTP, mDSN, ...)
  start_network_services();

  // We are ready to accept connections, notify on serial port
  serpr("Ready to serve requests on ");
  serpr(WiFi.localIP());
  serprln(" , waiting for connection...");
}

// Main firmware loop ... by default do nothing just start WEB server
void loop(void) {
  unsigned long currentMillis = millis();

  if (wifi_reconnect_pending) {
    wifi_reconnect_pending = false;
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }

  server.begin();

  for (int i=0; i<NUM_OUTPUTS; i++) {
    switch (exec_command[i]) {
      case 0:
         break;
      case 1:
        //serprln("Exec command FIBARO_ADD command " + i);
        fibaro_add(i);
        exec_command[i]=0;
        break;
      case 2:
        //serprln("Exec command ANGLE command " + i);
        motor[i].write(exec_parameter[i]);
        exec_command[i]=0;
        break;
      case 3:
        //serprln("Exec command TOGGLE command " + i);
        toggle(i);
        exec_command[i]=0;
        break;
      case 4:
        //serprln("Exec command AEOTEC_LIGHT_ADD, device " + i);
        aeotec_light_add(i);
        exec_command[i]=0;
        break;
      case 5:
        //serprln("Exec command AEOTEC_LIGHT_REMOVE, device " + i);
        aeotec_light_remove(i);
        exec_command[i]=0;
        break;
      case 6:
        //serprln("Exec command FIBARO_FLOOD_TRIGGER, device " + i);
        fibaro_flood_trigger(i);
        exec_command[i]=0;
        break;
      case 7:
        //serprln("Exec command VESTA-162_ADD_REMOVE, device " + i);
        vesta162(i);
        exec_command[i]=0;
        break;
      case 8:
        //serprln("Exec command CONTROL4_IDENTIFY, device " + i);
        control4_identify(i);
        exec_command[i]=0;
        break;
      case 9:
        //serprln("Exec command CONTROL4_FACTORY, device " + i);
        control4_factory_reset(i);
        exec_command[i]=0;
        break;
      case 10:
        //serprln("Exec command CONTROL4_CHANNEL, device " + i);
        control4_channel(i);
        exec_command[i]=0;
        break;
      case 11:
        //serprln("Exec command CONTROL4_REBOOT, device " + i);
        control4_reboot(i);
        exec_command[i]=0;
        break;
      case 12:
        //serprln("Exec command CONTROL4_LEAVE_MESH, device " + i);
        control4_leave_mesh(i);
        exec_command[i]=0;
        break;
      case 13:
        //serprln("Exec command CONTROL4_PUCK_IDENTIFY, device " + i);
        control4_puck_leave_mesh(i);
        exec_command[i]=0;
        break;
      case 999:
        //serprln("Exec command RESTART");
        config_save = true;
        esp_restart = true;
        exec_command[i]=0;
        break;
      default:
        exec_command[i]=0;
        break;        
    }
  }

  // Write configuration parameters if required
  if (config_save) {
    write_config();
    config_save = false;
  }

  // Restart device if required
  if (esp_restart && !config_save) {
    serprln("Please wait until device responds to HTTP request and restarts ...");
    delay(1000);
    Serial.flush();
    WiFi.setAutoReconnect(false);
    delay(1000);
    ESP.restart();
  }
  
  // Check if we are still conencted to the AP, reconnect if necessary
  /*
  if ((WiFi.status() != WL_CONNECTED)) {
    if  (currentMillis - previousMillis >= WIFI_RECONNECT_INTERVAL) {
      serprln("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis = currentMillis;
    }
  }
  */
}
