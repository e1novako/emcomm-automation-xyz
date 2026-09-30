#include <ESP8266WiFi.h>

#ifndef CONFIG_GPIO_CAP

#define MAX_CONFIG_BITS         6

#define GPIO_BIT(n)             ( 1 << n )
#define IS_GPIO_BIT_SET(b, n)   ( b && GPIO_BIT(n))
#define GPIO_CAPABILITY(u, n)   ( config_gpio_cap[u] & GPIO_BIT(n) )

enum CONFIG_GPIO_CAP {
    GPIO_CAPABILITY_READ,
    GPIO_CAPABILITY_WRITE,
    GPIO_CAPABILITY_TOGGLE,
    GPIO_CAPABILITY_INCLUDE,
    GPIO_CAPABILITY_EXCLUDE,
    GPIO_CAPABILITY_FACTORY_RESET,
    GPIO_CAPABILITY_6,
    GPIO_CAPABILITY_7,
    GPIO_CAPABILITY_8,
    GPIO_CAPABILITY_9,
    GPIO_CAPABILITY_10,
    GPIO_CAPABILITY_11,
    GPIO_CAPABILITY_12,
    GPIO_CAPABILITY_13,
    GPIO_CAPABILITY_14,
    GPIO_CAPABILITY_15,
    GPIO_CAPABILITY_16,
    GPIO_CAPABILITY_17,
    GPIO_CAPABILITY_18,
    GPIO_CAPABILITY_19,
    GPIO_CAPABILITY_20,
    GPIO_CAPABILITY_21,
    GPIO_CAPABILITY_22,
    GPIO_CAPABILITY_23,
    GPIO_CAPABILITY_24,
    GPIO_CAPABILITY_25,
    GPIO_CAPABILITY_CUSTOM_COMMANDS,
    GPIO_CAPABILITY_OUTPUT,
    GPIO_CAPABILITY_MANUFACTURER
};
#endif

#ifdef C4NODEMCU_MAIN
// Parameters sent to the web server
const char* PARAM_INPUT_1 = "relay";  
const char* PARAM_INPUT_2 = "state";
const char* PARAM_INPUT_3 = "actuator";  
const char* PARAM_INPUT_4 = "function";
const char* PARAM_INPUT_6 = "outputs";
const char* PARAM_INPUT_7 = "read";
const char* PARAM_INPUT_8 = "write";


const char* PARAM_INPUT_DEVICE_TYPE       = "device_type";
const char* PARAM_INPUT_SHOW_DESCRIPTION  = "show_description";
const char* PARAM_INPUT_DEBUG_ENABLED     = "debug_enabled";

const char* PARAM_INPUT_GPIO              = "cg_";
const char* PARAM_INPUT_GPIO_CAPABILITY   = "cp_";
const char* PARAM_INPUT_MIN_ANGLE         = "min_";
const char* PARAM_INPUT_MAX_ANGLE         = "max_";
const char* PARAM_INPUT_MANUFACTURER      = "mnf_";
const char* PARAM_INPUT_DESCRIPTION       = "d_";

const char *PARAM_GPIO_EXPANDER_TYPE      = "useexp";

const char* PARAM_SSID  = "SSID";
const char* PARAM_PASS  = "PASS";
const char* PARAM_RFPOWER  = "RFPOWER";
const char* PARAM_MAC   = "MAC";

// GPIO Capability Names
String CONFIG_GPIO_CAP_NAME[] = {
    "Read",
    "Set / Write",
    "Toggle",
    "Include",
    "Exclude",
    "Factory reset",
    "6",
    "7",
    "8",
    "9",
    "10",
    "11",
    "12",
    "13",
    "14",
    "15",
    "16",
    "17",
    "18",
    "19",
    "20",
    "21",
    "22",
    "23",
    "24",
    "25",
    "Custom commands",
    "GPIO dir. output",
    "Manufacturer"
};

// Configuration parameters
String  config_device_type;
String  config_gpio_used_number     = "8";
String  config_gpio_use_expander    = "0";

uint32_t DFGPC = GPIO_BIT(GPIO_CAPABILITY_READ); // GPIO_BIT(GPIO_CAPABILITY_READ) | GPIO_BIT(GPIO_CAPABILITY_WRITE) | GPIO_BIT(GPIO_CAPABILITY_GPIO_DIRECTION_OUTPUT );

uint32_t config_gpio_cap[MAX_GPIOS] = { DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC, DFGPC };

String  config_gpio[MAX_GPIOS]      = { "16",  "5",  "4",  "0",  "2", "14", "12", "13",  "8",  "9", "10", "11", "12", "13", "14", "15" };
String  min_angle[MAX_GPIOS]        = { "15", "10", "10", "14", "12", "12", "12", "12", "12", "12", "12", "12", "12", "12", "12", "12" };
String  max_angle[MAX_GPIOS]        = { "40", "40", "40", "40", "40", "40", "40", "40", "40", "40", "40", "40", "40", "40", "40", "40" };

String  config_show_description;
String  config_manufacturer[MAX_GPIOS];
String  config_description[MAX_GPIOS];

// Runtime debug-output toggle; persisted in config and toggleable from the web UI Settings page.
bool debug_enabled = true;

uint8_t oapos[MAX_GPIOS]            = { 180,  180,   180,  180,  180,  180,  180,  180, 180,  180,   180,  180,  180,  180,  180,  180 };
uint8_t exec_command[MAX_GPIOS]     = {   0,    0,     0,    0,    0,    0,    0,    0,   0,    0,     0,    0,    0,    0,    0,    0 };
int exec_parameter[MAX_GPIOS]       = {   0,    0,     0,    0 ,   0,    0,    0,    0,   0,    0,     0,    0,    0,    0,    0,    0 };

String  valid_gpio[]                = {  "0",  "2",   "4",  "5", "12", "13", "14", "15", "16", "0",  "0",  "0",  "0",  "0",  "0",  "0" };
Servo motor[MAX_GPIOS];

bool config_save = false;
bool factory_default = false;

int progress  = -1;

// Default parameters ... can be configured from configuration page
String ssid = "Z-Wave Automation";
String password = "Fiber714Cvet";
String rf_power = "20";

unsigned long previousMillis = 0;
uint8_t MAC[6];

// Should we restart device?
bool esp_restart=false;

// Append this to default NodeMCU hostname 
String newHostname = "";

extern const char page_restart_device[];

// Default port for the HTTP server is 80
AsyncWebServer server(80);

// I2C port expander
PCF8575 pcf8575(0x20);
#else
// ----------------------------------------------------------------------------------------------------------------------------------------------
// Configuration parameters
extern String   config_device_type, config_gpio_used_number, config_show_description, config_gpio_use_expander, ssid, password, rf_power, newHostname;
extern String   config_manufacturer[MAX_GPIOS], config_description[MAX_GPIOS], valid_gpio[MAX_GPIOS], config_gpio[MAX_GPIOS];
extern String   min_angle[MAX_GPIOS], max_angle[MAX_GPIOS];
extern String   CONFIG_GPIO_CAP_NAME[];
extern uint32_t config_gpio_cap[MAX_GPIOS];
extern uint8_t  oapos[MAX_GPIOS], exec_command[MAX_GPIOS];
extern bool     config_save, factory_default, esp_restart;
extern bool     debug_enabled;
extern int      exec_parameter[MAX_GPIOS], progress;
extern Servo    motor[MAX_GPIOS];
extern unsigned long previousMillis;
extern uint8_t  MAC[6];

extern  const char    *PARAM_INPUT_1, *PARAM_INPUT_2, *PARAM_INPUT_3, *PARAM_INPUT_4, *PARAM_INPUT_6;
extern  const char    *PARAM_INPUT_GPIO, *PARAM_INPUT_MIN_ANGLE, *PARAM_INPUT_MAX_ANGLE;
extern  const char    *PARAM_INPUT_DEVICE_TYPE, *PARAM_INPUT_SHOW_DESCRIPTION, *PARAM_INPUT_DEBUG_ENABLED;
extern  const char    *PARAM_INPUT_MANUFACTURER, *PARAM_INPUT_DESCRIPTION;
extern  const char    *PARAM_SSID, *PARAM_MAC, *PARAM_PASS, *PARAM_RFPOWER, *PARAM_GPIO_EXPANDER_TYPE;
extern  const char    *PARAM_INPUT_GPIO_CAPABILITY;

extern const char page_restart_device[];

extern AsyncWebServer server;
extern PCF8575        pcf8575;
#endif
