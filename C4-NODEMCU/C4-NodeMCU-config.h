// Filesystem support for configuration parameter save/restore
#include <LittleFS.h>

extern void setup_outputs(String devt);
extern String macToString(const unsigned char* mac);

// Erase all parmeters from filesystem
void erase_parameters() {
  serprln("Erasing filesystem and all configured parameters.\n");
  LittleFS.format();
}

// Read file from NodeMCU filesystem
String file_read(String file_name) {
  String result = "";
  
  File this_file = LittleFS.open(file_name, "r");
  if (!this_file) { // failed to open the file, retrn empty result
    return result;
  }
  while (this_file.available()) {
      result += (char)this_file.read();
  }
  
  this_file.close();
  return result;
}

// Write file to NodeMCU filesystem
bool file_write(String file_name, String contents) {  
  File this_file = LittleFS.open(file_name, "w");
  if (!this_file) { // failed to open the file, return false
    return false;
  }
  int bytesWritten = this_file.print(contents);
 
  if (bytesWritten == 0) { // write failed
      return false;
  }
   
  this_file.close();
  return true;
}

// Read configuration files and store them to variables
void read_config() {
  int erase=0;
  
  LittleFS.begin();

  serprln("Check if FLASH button is pressed?\n");
  pinMode(0, INPUT_PULLUP);
  delay(100);
  for (int i=0; i<120; i++)
    if (!digitalRead(0))
      erase++;
  if (erase>100) {
    serprln("Requested deletion of configuration parameters!");
    erase_parameters();
  }

  if (file_read(PARAM_SSID) != "" && file_read(PARAM_PASS) != "") {
    ssid = file_read(PARAM_SSID);
    password = file_read(PARAM_PASS);
    serprln("SSID: " + ssid);
    serprln("PASS: " + password + "\n");
  } else {
    serprln("Saving default SSID and Passphrase to default parameters...\n");
    config_save=true;
  }

  if (file_read(PARAM_RFPOWER) != "" && file_read(PARAM_RFPOWER) != "") {
    rf_power = file_read(PARAM_RFPOWER);
    serprln("RF POWER: " + rf_power);
  } else {
    config_save=true;
  }

  if (file_read(PARAM_MAC) != "") {
    String sMAC = file_read(PARAM_MAC);

    char *newMAC;
    newMAC = (char*)malloc(20);
    uint8_t parsedMAC[6];

    sMAC.toCharArray(newMAC, 20);
    newMAC[19]=0;

    sscanf(newMAC, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx", &parsedMAC[0], &parsedMAC[1], &parsedMAC[2], &parsedMAC[3], &parsedMAC[4], &parsedMAC[5]);

    for (int ii=0; ii<6; ii++)
      MAC[ii]=parsedMAC[ii];

    free(newMAC);

    serprln("MAC:  " + macToString(MAC) + "\n");
  } else {
    serprln("Saving default MAC to default parameters...\n");
    config_save=true;
  }

  config_device_type = file_read(PARAM_INPUT_DEVICE_TYPE);
  if (config_device_type == "") {
    config_device_type = DEVICE_TYPE_RELAY;
    config_save=true;
  } else {
    //String ffd = file_read("factory_default");
    //if (ffd == "YES") factory_default = true;
    file_write("factory_default", "NO");

    config_gpio_use_expander = file_read(PARAM_GPIO_EXPANDER_TYPE);
    serprln("Use GPIO expander:\t" + config_gpio_use_expander);
    if (config_gpio_use_expander!="0" && config_gpio_use_expander!="1" && config_gpio_use_expander!="2")
        config_gpio_use_expander="0";

    config_gpio_used_number = file_read("gpio_used_number");
    int helper = config_gpio_used_number.toInt();
    if (helper < 0) helper = 0;
    if (helper > MAX_GPIOS) helper = MAX_GPIOS;
    config_gpio_used_number = String(helper);

    serprln("Used GPIO's:\t\t" + config_gpio_used_number);
    config_show_description = file_read(PARAM_INPUT_SHOW_DESCRIPTION);
    serprln("Show description:\t" + config_show_description);

    {
      String debug_str = file_read(PARAM_INPUT_DEBUG_ENABLED);
      debug_enabled = (debug_str == "" || debug_str == "1");
      serprln("Debug enabled:\t" + String(debug_enabled ? "1" : "0"));
    }
    delay(100);

    for (int n=0; n<NUM_OUTPUTS; n++) {
      config_gpio[n]  = file_read("config_gpio_output_" + n);
      String gpio_cap = file_read("config_gpio_capability_" + n);
      config_gpio_cap[n] = gpio_cap.toInt();

      min_angle[n]    = file_read("config_min_angle_" + n);
      max_angle[n]    = file_read("config_max_angle_" + n);
      if (max_angle[n] == "")
         max_angle[n] = String(DEFAULT_MAX_ANGLE);

      config_manufacturer[n] = file_read("config_manufacturer_" + n);
      if (config_manufacturer[n] == "")
         config_manufacturer[n] = DEFAULT_MANUFACTURER;

      config_description[n] = file_read("config_description_" + n);
      if (config_description[n] == "")
         config_description[n] = DEFAULT_DESCRIPTION;

      if (false) {
        serprln("Output: " + String(n));
        serprln("   GPIO:         " + config_gpio[n]);
        serprln("   GPIO CAP:     " + String(config_gpio_cap[n]));
        serprln("   MIN_ANGLE:    " + min_angle[n]);
        serprln("   MAX_ANGLE:    " + max_angle[n]);
        serprln("   Manufacturer: " + config_manufacturer[n]);
        serprln("   Description:  " + config_description[n]);
      }
      delay(100);
    }
  }

  if (config_device_type == DEVICE_TYPE_RELAY || config_device_type == DEVICE_TYPE_ACTUATOR) {
    serprln("Device type: " + config_device_type);
    delay(500);
  } else {
    serprln("Unknown Device type, erasing file system...");
    delay(500);
    // Format filesystem to test default parameters
    LittleFS.format();
    esp_restart = true;
  }

  // ---------------------------------------------------------------------------------------------------------- //
}

// Write parameters to filesystem on device (each parameter is in it's own file)
void write_config() {
  serprln("\nSaving configuration options - begin");

  serprln("Connection parameters ...");
  file_write(PARAM_SSID, ssid);
  file_write(PARAM_PASS, password);
  file_write(PARAM_RFPOWER, rf_power);

  file_write(PARAM_MAC, macToString(MAC));

  serprln("Use GPIO expander:\t" + config_gpio_use_expander);
  file_write(PARAM_GPIO_EXPANDER_TYPE, config_gpio_use_expander);

  serprln("Device_type:\t" + config_device_type);
  file_write(PARAM_INPUT_DEVICE_TYPE, config_device_type);

  serprln("Factory_default:\t" + factory_default);
  file_write("factory_default", factory_default ? "YES" : "NO");

  serprln("Show description:\t" + config_show_description);
  file_write(PARAM_INPUT_SHOW_DESCRIPTION, config_show_description);

  serprln("Debug enabled:\t" + String(debug_enabled ? "1" : "0"));
  file_write(PARAM_INPUT_DEBUG_ENABLED, debug_enabled ? "1" : "0");

  serprln("\nUsed GPIOs:\t" + config_gpio_used_number);
  file_write("gpio_used_number", config_gpio_used_number);

  for (int n=0; n<NUM_OUTPUTS; n++) {
    //serprln("   GPIO #" + String(n) + " = " + config_gpio[n]);
    file_write("config_gpio_output_" + n, config_gpio[n]);
    //serprln("   GPIO #" + String(n) + " = " + config_gpio_cap[n]);
    file_write("config_gpio_capability_" + n, String(config_gpio_cap[n]));
    //serprln("   min_angle_" + String(n) + " = " + min_angle[n] );
    file_write("config_min_angle_" + n, min_angle[n]);
    //serprln("   max_angle_" + String(n) + " = " + max_angle[n] );
    file_write("config_max_angle_" + n, max_angle[n]);

    //serprln("   config_manufacturer_" + String(n) + " = " + config_manufacturer[n] );
    file_write("config_manufacturer_" + n, config_manufacturer[n]);
    //serprln("   config_decription_" + String(n) + " = " + config_description[n] );
    file_write("config_description_" + n, config_description[n]);
  }

  setup_outputs(config_device_type);

  serprln("\nSaving configuration options - end");

  delay(1000);

  serprln("\nSaving configuration options - continue");

}
