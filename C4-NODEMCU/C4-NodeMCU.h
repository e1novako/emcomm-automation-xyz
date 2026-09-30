#include "C4-NodeMCU-commands.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/Diagnostics.h"

// ESP8266
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>

// WEB server library for ESP chips
#include <ESPAsyncWebServer.h>

// Filesystem support for configuration parameter save/restore
#include <LittleFS.h>

// Servo support
#include <Servo.h>

// Port expander library
#include <PCF8575.h>

// Software version
#define SW_VERSION            "v3.2.3"

// Debugging to serial port... toggleable at runtime from the web UI Settings page (debug_enabled), disabled by default via DEBUG_DEFAULT if desired.
#define DEBUG true

#define serpr(a...) emcomm::debugIfEnabled(DEBUG && debug_enabled, [&]() { Serial.print(a); })
#define serprf(a...) emcomm::debugIfEnabled(DEBUG && debug_enabled, [&]() { Serial.printf(a); })
#define serprln(a...) emcomm::debugIfEnabled(DEBUG && debug_enabled, [&]() { Serial.println(a); })

// Most of our automation devices have boards with 8 relays
#define MAX_GPIOS             16

// Number of configured outputs, read from configuration
#define NUM_OUTPUTS           config_gpio_used_number.toInt()

// Maximum allowed angle for motor actuators
#define DEFAULT_MAX_ANGLE     40

#define DEFAULT_MANUFACTURER  "--"
#define DEFAULT_DESCRIPTION   "--"

// Possible device types:
#define DEVICE_TYPE_RELAY     "C4-RELAY-"
#define DEVICE_TYPE_ACTUATOR  "C4-ACTUATOR-"

#define GPIO_EXPANDER_NO      "0"
#define GPIO_EXPANDER_PCF8574 "1"
#define GPIO_EXPANDER_PCF8575 "2"

// If set to true we are using NO (Normaly Open) relay contacts
#define RELAY_NO    true

#define WIFI_RECONNECT_INTERVAL 10000

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

#include "C4-NodeMCU-variables.h"
#include "C4-NodeMCU-functions.h"

extern void erase_parameters();
