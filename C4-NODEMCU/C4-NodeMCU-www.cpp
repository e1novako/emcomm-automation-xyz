#define C4NODEMCU_WWW

#include "C4-NodeMCU.h"
#include "C4-NodeMCU-WWW.h"

String add_checkbox(String name, boolean checked) {
  return "<input type='checkbox' name='" + name + "' " + (checked ? "checked" : "") + ">";
}

// Require HTTP Basic Auth (admin / current Wi-Fi password) for sensitive endpoints.
// Returns true if authenticated; otherwise sends a 401 challenge and returns false.
bool require_auth(AsyncWebServerRequest *request) {
  if (!request->authenticate("admin", password.c_str())) {
    request->requestAuthentication();
    return false;
  }
  return true;
}

char *uptime() {
char *tme=(char*)malloc(16);
uint32_t  upt = millis() / 1000;
uint8_t   sec, min;
int       hrs;

hrs = upt/3600;
min = (upt-hrs*3600)/60;
sec = upt-hrs*3600-min*60;

snprintf(tme, 16, "%02d:%02d:%02d", hrs, min, sec);

return tme;
}

String macToString(const unsigned char* mac) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// Replaces placeholder with button section in your web page
String processor(const String &var){
  String buttons = "";
    if(var == "NUM_OUTPUTS") {
        buttons += NUM_OUTPUTS;
    } else if (var == "HOSTNAME") {
        buttons += newHostname;
    } else if (var == "SW_VERSION") {
        buttons += SW_VERSION;
    } else if (var == "MACADDR") {
        buttons += macToString(MAC);
    } else if (var == "UPTIME") {
        char *tme=uptime();
        buttons += String(tme);
        free(tme);
        buttons += "<script>var uptime=" + String(millis()/1000) + "</script>";
    } else if (var == "WIFI_OPTIONS") {
        buttons += "SSID:&nbsp;&nbsp;<input type='text' name='" + String(PARAM_SSID) + "' size='32' value='" + String(ssid) + "'><BR>\n";
        buttons += "PASS:&nbsp;<input type='password' name='" + String(PARAM_PASS) + "' size='32' value='" + String(password) + "'><BR>\n";
        buttons += "RF Power:&nbsp;<input type='number' min='0' max='20' name='" + String(PARAM_RFPOWER) + "' size='2' value='" + String(rf_power) + "'><BR>\n";        
    } else if (var =="TITLE") {
        IPAddress ipA = WiFi.localIP();
        buttons += "  <title>C4-NodeMCU - " + String(ipA[2]) + "." + String(ipA[3]) + "</title>\n";
        buttons += "<link rel=\"icon\" type=\"image/x-icon\" href=\"/c4logo.svg\">\n";
    } else if (var == "GPIO_PIN_STATES") {
        for (int i=1; i<=NUM_OUTPUTS; i++) {
          String relayStateValue = "";
          buttons += "   \"port_" + String(i-1) + "\": ";
          if (GPIO_CAPABILITY(i-1, GPIO_CAPABILITY_READ)) {
            relayStateValue = relayState(i);
            if (relayStateValue == "checked") {
              buttons += "true";
            } else {
              buttons += "false";
            }
          } else {
            buttons += "-1";
          }
          buttons += ",\n";
        }
        buttons += "   \"uptime\": " + String(millis()/1000);
        /*
        buttons += "   \"uptime\": \"";
        char *tme=uptime();
        buttons += String(tme);
        free(tme);
        buttons += "\"";
        */
    } else if (var == "CONFIG_OPTIONS") {
        buttons += "Device type: \n<select name='";
        buttons += PARAM_INPUT_DEVICE_TYPE;
        buttons += "'>";
        buttons += " <option value='";
        buttons += DEVICE_TYPE_RELAY;
        buttons += "'";
        if (config_device_type == DEVICE_TYPE_RELAY)
          buttons += " selected";
        buttons += ">Relay Controller</option>\n";        
        buttons += " <option value='";
        buttons += DEVICE_TYPE_ACTUATOR;
        buttons += "'";
        if (config_device_type == DEVICE_TYPE_ACTUATOR)
          buttons += " selected";
        buttons += ">Actuator Controller</option>\n</select><BR><BR>\n";


        buttons += "Use GPIO expander: \n<select name='";
        buttons += PARAM_GPIO_EXPANDER_TYPE;
        buttons += "'>";
        buttons += " <option value='";
        buttons += GPIO_EXPANDER_NO;
        buttons += "'";
        if (config_gpio_use_expander == GPIO_EXPANDER_NO)
          buttons += " selected";
        buttons += ">No GPIO Expander</option>\n";        
        buttons += " <option value='";
        buttons += GPIO_EXPANDER_PCF8574;
        buttons += "'";
        if (config_gpio_use_expander == GPIO_EXPANDER_PCF8574)
          buttons += " selected";
        buttons += ">GPIO Expander PCF8574</option>\n";
        buttons += " <option value='";
        buttons += GPIO_EXPANDER_PCF8575;
        buttons += "'";
        if (config_gpio_use_expander == GPIO_EXPANDER_PCF8575)
          buttons += " selected";
        buttons += ">GPIO Expander PCF8575</option>\n</select><BR><BR>\n";


        buttons += "GPIO outputs: <input name='outputs' type='text' size='2' min='1' max='";
        buttons += MAX_GPIOS;
        buttons += "' value='" + config_gpio_used_number + "'>\n";

        buttons += "<BR><BR>\n";
        buttons += "Debug output: \n<select name='";
        buttons += PARAM_INPUT_DEBUG_ENABLED;
        buttons += "'>\n";
        buttons += " <option value='0' ";
        if (!debug_enabled)
          buttons += " selected";
        buttons += ">disabled</option>\n";
        buttons += " <option value='1' ";
        if (debug_enabled)
          buttons += " selected";
        buttons += ">enabled</option>\n</select>\n";

        if (config_device_type == DEVICE_TYPE_ACTUATOR) {
          buttons += "<BR><BR>\n";
  
          buttons += "Minimum angle:<BR>\n";
          for (int i=0; i<config_gpio_used_number.toInt() && i<MAX_GPIOS; i++) {
            buttons += "<input type='text' size='2' min='0' max='" + max_angle[i] + "' name='";
            buttons += PARAM_INPUT_MIN_ANGLE;
            buttons += i;
            buttons += "' value='";
            buttons += min_angle[i];
            buttons += "'>\n";
          }

          buttons += "<BR><BR>\n";
          buttons += "Maximum angle:<BR>\n";
          for (int i=0; i<config_gpio_used_number.toInt() && i<MAX_GPIOS; i++) {
            buttons += "<input type='text' size='2' min='" + min_angle[i] + "' max='90' name='";
            buttons += PARAM_INPUT_MAX_ANGLE;
            buttons += i;
            buttons += "' value='";
            buttons += max_angle[i];
            buttons += "'>\n";
          }
        }
     } else if (var == "CONFIG_DESCRIPTION") {
        buttons += "Description:\n<select name='";
        buttons += PARAM_INPUT_SHOW_DESCRIPTION;
        buttons += "'>\n";
        buttons += " <option value='0' ";
        if (config_show_description == "0")
          buttons += " selected";
        buttons += ">hide</option>\n";
        buttons += " <option value='1' ";
        if (config_show_description == "1")
          buttons += " selected";
        buttons += ">show</option>\n</select><BR><BR>\n";

        buttons += "<TABLE class='table table-header-rotated'><thead>";
        buttons += "<TR><th class='rotate'><div><span>I/O Number</span></div></th>\n<th class='rotate'><div><span>Manufacturer</span></div></th>\n<th class='rotate'><div><span>Device Model</span></div></th>\n";
        buttons += "<th class='rotate'><div><span>Description</span></div></th>\n<th class='rotate'><div><span>GPIO Number</span></div></th>\n";
        for (int j=0; j<GPIO_CAPABILITY_MANUFACTURER  && j<MAX_CONFIG_BITS; j++) {
            buttons += "<th class='rotate'><div><span>";
            buttons += CONFIG_GPIO_CAP_NAME[j];
            buttons += "</span></div></th>\n";
        }
        for (int j=26; j<=27; j++) {
            buttons += "<th class='rotate'><div><span>";
            buttons += CONFIG_GPIO_CAP_NAME[j];
            buttons += "</span></div></th>\n";
        }
          
        buttons += "</TR></thead><tbody>";
        for (int i=0; i<config_gpio_used_number.toInt() && i<MAX_GPIOS; i++) {
          buttons += "<TR>\n<TD>" + String(i+1) + "</TD>";
          
          buttons += "<TD> </TD>\n";

          buttons += "<TD><input type='text' size='12' name='";
          buttons += PARAM_INPUT_MANUFACTURER;
          buttons += i;
          buttons += "' value='";
          buttons += config_manufacturer[i];
          buttons += "'></TD>\n";

          buttons += "<TD><input type='text' size='12' name='";
          buttons += PARAM_INPUT_DESCRIPTION;
          buttons += i;
          buttons += "' value='";
          buttons += config_description[i];
          buttons += "'></TD>\n";

          buttons += "<TD><input type='text' size='2' min='0' max='15' name='";
          buttons += PARAM_INPUT_GPIO;
          buttons += i;
          buttons += "' value='";
          buttons += config_gpio[i];
          buttons += "'></TD>\n";

          for (int j=0; j<GPIO_CAPABILITY_MANUFACTURER && j<MAX_CONFIG_BITS; j++) {
              buttons += "<TD><center>";
              String inn = PARAM_INPUT_GPIO_CAPABILITY; inn+=i; inn += "_"; inn += j;
              buttons += add_checkbox(inn, GPIO_CAPABILITY(i, j) );
              buttons += "</TD>\n";
          }
          for (int j=26; j<=27; j++) {
              buttons += "<TD><center>";
              String inn = PARAM_INPUT_GPIO_CAPABILITY; inn+=i; inn += "_"; inn += j;
              buttons += add_checkbox(inn, GPIO_CAPABILITY(i, j) );
              buttons += "</TD>\n";
          }

          buttons += "</TR>\n";
        }
        buttons += "</TR></tbody></TABLE>";
    } else if(var == "PLACEHOLDER") {
      if (config_device_type == DEVICE_TYPE_RELAY) {
        int i, z;
        for (z = 1; z < 33; z += 8) {

          int kraj = ( z + 8 > NUM_OUTPUTS ? NUM_OUTPUTS : z + 7);

          if (z <= kraj)
            buttons += "<TR>\n";
          for(i = z; i <= kraj; i++) {
            String relayStateValue = "off";
            if (GPIO_CAPABILITY(i-1, GPIO_CAPABILITY_READ)) {
              relayStateValue = relayState(i);
            }
            buttons += "    <td>\n";
            if (config_show_description == "0") {
              buttons += "     <h4>Relay #" + String(i) + "<br>";
              buttons += "GPIO " + config_gpio[i-1];
            } else {
              buttons += "     <h4>" + config_manufacturer[i-1] + "<br>";
              buttons += config_description[i-1];
            }
            if (GPIO_CAPABILITY(i-1, GPIO_CAPABILITY_WRITE)) {
              buttons +="</h4>\n     <label class=\"switch\"><input type=\"checkbox\" onchange=\"sRly(this)\" id=\"port_" + String(i-1) + "\" "+ relayStateValue +"><span class=\"slider\"></span></label>";
            } else {
              buttons +="</h4>\n     <label class=\"switch\"><input disabled type=\"checkbox\" id=\"port_" + String(i-1) + "\" "+ relayStateValue +"><span class=\"slider\"></span></label>";
            }
            buttons += "\n    </td>\n";
          }
          if (z <= kraj)
            buttons += "   </TR>\n   <TR>\n";
          for(i = z; i <= kraj; i++) {
            buttons += "    <td>\n";
            if (GPIO_CAPABILITY(i-1, GPIO_CAPABILITY_TOGGLE)) {
              buttons +="    </h4><input type=\"button\" onclick=\"rCmd(this, '" + String(COMMAND_TOGGLE) + "')\" id=\"t_" + String(i-1) + "\" value=\"Toggle #" + String(i) + "\"><BR><BR>\n";
            }
            if (GPIO_CAPABILITY(i-1, GPIO_CAPABILITY_CUSTOM_COMMANDS)) {
              buttons +="    </h4><input type=\"button\" onclick=\"rCmd(this, '" + String(COMMAND_CONTROL4_IDENTIFY) + "')\" id=\"id_" + String(i-1) + "\" value=\"C4-Identify #" + String(i) + "\"><BR><BR>\n";
              buttons +="    </h4><input type=\"button\" onclick=\"rCmd(this, '" + String(COMMAND_FIBARO_ADD) + "')\" id=\"a_" + String(i-1) + "\" value=\"Fibaro Incl #" + String(i) + "\"><BR><BR>\n";
              buttons +="    </h4><input type=\"button\" onclick=\"rCmd(this, '" + String(COMMAND_FIBARO_TRIG_FLOOD) + "')\" id=\"fl_" + String(i-1) + "\" value=\"Fibaro Flood #" + String(i) + "\"><BR><BR>\n";
              buttons +="    </h4><input type=\"button\" onclick=\"rCmd(this, '" + String(COMMAND_AEOTEC_LIGHT_REMOVE) + "')\" id=\"ar_" + String(i-1) + "\" value=\"AeoLght Excl #" + String(i) + "\"><BR><BR>\n";
            }
            buttons += "    </td>\n";
          }
          if (z <= kraj)
            buttons += "   </TR>\n";
        }
      } else if (config_device_type == DEVICE_TYPE_ACTUATOR) {
        for(int i=1; i<=NUM_OUTPUTS && i<MAX_GPIOS; i++) {
           String motorStateValue = "0";
           if (config_show_description == "0") {
             buttons +="<td>\n  ";
             //buttons += "<h4>Actuator #" + String(i) + "<br>";
             buttons +="GPIO " + config_gpio[i-1];
             buttons += "<BR>";
           } else {
             buttons += "<td>\n  <h4>" + config_manufacturer[i-1] + "<br>";
             buttons += config_description[i-1];
           }
           buttons +="\n     <BR></h4><input type=\"button\" onclick=\"mtr(this, '" + String(COMMAND_FIBARO_ADD) +"')\" id=\"" + String(i-1) + "\" value=\"Fibaro Add #" + String(i) + "\"><BR>";
           buttons +="\n     <BR></h4><input type=\"button\" onclick=\"mtr(this, '" + String(COMMAND_TOGGLE) + "')\" id=\"" + String(i-1) + "\" value=\"Toggle #" + String(i) + "\"><BR>";
           buttons +="\n     <BR><input type=\"text\" onmouseleave=\"aAngl(this)\" id=\"" + String(100+i-1) + "\" value=\"" + String(oapos[i-1]) + "\" min=\"10\" max=\"" + max_angle[i-1] + "\" size=\"2\">";
           buttons +="\n</td>\n";
        }
      }
    } else if (var == "REBOOT_MESSAGE") {
        if (factory_default) {
          for (int i=0; i<4; i++)
            buttons += "<BR>";
          buttons += "Please wait until device reboots<br>and reconnects to network...";
        }
    } else if(var == "PLACEHOLDER") {
        buttons += "<BR>Unknown device type<BR>";
    }
  //serprln("INFO: HTML page length " + String(buttons.length()) + " bytes.");
  return buttons;
}

String processor_js(const String& var){
  String buttons = "";
  if (var == "AUTOMATIC_REBOOT") {
    if (factory_default) {
      buttons += "setTimeout(function() { window.location.reload(); }, 15000);";
      buttons += "var result = document.getElementById('table'); result.style.display = 'none';\n";
      buttons += "var blink = document.getElementById('blink');";
      buttons += "setInterval(function() { blink.style.opacity = (blink.style.opacity == 0 ? 1 : 0); }, 1000);\n";
    }
  }
  return buttons;
}

// Start configured network services
void start_network_services() {
  // Start the MDNS - required for OTA
  MDNS.begin(WiFi.hostname().c_str());

  // How should we handle different URL's:
  // Root or Home Page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: Root WEB page...");
    request->send_P(200, "text/html", index_html, processor);
  });

  // /update - Change relay state
  if (config_device_type == DEVICE_TYPE_RELAY) {
    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
      String inputMessage,  inputParam;
      String inputMessage2, inputParam2;

      if (request->hasParam(PARAM_INPUT_1)) {
        inputMessage = request->getParam(PARAM_INPUT_1)->value();
        inputParam = PARAM_INPUT_1;
        int relayIndex = inputMessage.toInt();
        if (relayIndex < 0 || relayIndex >= MAX_GPIOS || relayIndex >= NUM_OUTPUTS) {
          serprln("Invalid relay index: " + inputMessage);
          request->send(400, "text/plain", "Invalid output index");
          return;
        }
        //serprln("Found param1 " + inputParam);
        if (request->hasParam(PARAM_INPUT_2)) {
            inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
            inputParam2 = PARAM_INPUT_2;
            //serprln("Found param2 " + inputParam2);
            if(RELAY_NO) {
              //serpr("NO ");
              gpioSet(inputMessage.toInt(), !inputMessage2.toInt());
            } else {
              //serpr("NC ");
              gpioSet(inputMessage.toInt(), inputMessage2.toInt());
            }
            //serprln(inputMessage + inputMessage2);
            request->send(200, "text/plain", "OK");
        } else if (request->hasParam(PARAM_INPUT_4)) {
          inputMessage2 = request->getParam(PARAM_INPUT_4)->value();
          inputParam2 = PARAM_INPUT_4;
          //serprln("Found param3 " + inputParam2);
          if (inputMessage2 == COMMAND_TOGGLE) {
            //serprln("toggle: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=3;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_FIBARO_ADD || \
                inputMessage2 == COMMAND_ZEN71_ADD || inputMessage2 == COMMAND_ZEN71_REMOVE || \
                inputMessage2 == COMMAND_ZEN72_ADD || inputMessage2 == COMMAND_ZEN72_REMOVE || \
                inputMessage2 == COMMAND_ZEN76_ADD || inputMessage2 == COMMAND_ZEN76_REMOVE || \
                inputMessage2 == COMMAND_ZEN77_ADD || inputMessage2 == COMMAND_ZEN77_REMOVE
                ) {
            //serprln("fibaro_add: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=1;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_FIBARO_TRIG_FLOOD) {
            //serprln("fibaro_flood: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=6;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_VESTA162_ADD || inputMessage2 == COMMAND_VESTA162_REMOVE) {
            //serprln("vesta162_cmd: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=7;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_CONTROL4_IDENTIFY) {
            //serprln("control4_identify: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=8;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_CONTROL4_PUCK_LEAVE_MESH) {
            //serprln("control4_puck_identify: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=13;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_CONTROL4_FACTORY_RESET) {
            //serprln("control4_factory_reset_cmd: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=9;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_CONTROL4_CHANNEL) {
            //serprln("control4_channel_cmd: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=10;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_CONTROL4_REBOOT) {
            //serprln("control4_reboot_cmd: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=11;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_CONTROL4_LEAVE_MESH) {
            //serprln("control4_leave_mesh_cmd: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=12;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_AEOTEC_LIGHT_ADD) {
            //serprln("aeotec_light_add: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=4;
            request->send(200, "text/plain", "OK");
          } 
          else if (inputMessage2 == COMMAND_AEOTEC_LIGHT_REMOVE) {
            //serprln("aeotec_light_remove: " + inputMessage + " -- " + inputMessage2);
            exec_command[inputMessage.toInt()]=5;
            request->send(200, "text/plain", "OK");
          } else {
            serprln("Unknown function " + inputMessage2);
            http_redirect(request, "/", "1", "Go West");
          }
        } else {
            serprln("Additional param not found");
            http_redirect(request, "/", "1", "Go East");
        }
      }
   });
  }

  // /update - Change motor angle, only if device type is C4-ACTUATOR
  if (config_device_type == DEVICE_TYPE_ACTUATOR) {
    server.on("/actuator", HTTP_GET, [] (AsyncWebServerRequest *request) {
      String inputMessage,  inputParam;
      String inputMessage2, inputParam2;

      if (request->hasParam(PARAM_INPUT_3) & request->hasParam(PARAM_INPUT_4)) {
        inputMessage = request->getParam(PARAM_INPUT_3)->value();
        inputParam = PARAM_INPUT_3;
        inputMessage2 = request->getParam(PARAM_INPUT_4)->value();
        inputParam2 = PARAM_INPUT_4;
        int actuatorIndex = inputMessage.toInt();
        if (actuatorIndex < 0 || actuatorIndex >= MAX_GPIOS || actuatorIndex >= NUM_OUTPUTS) {
          serprln("Invalid actuator index: " + inputMessage);
          request->send(400, "text/plain", "Invalid output index");
          return;
        }
        if (inputMessage2 == COMMAND_FIBARO_ADD) {
          serprln("fibaro_add: " + inputMessage + " -- " + inputMessage2);
          exec_command[inputMessage.toInt()]=1;
          request->send(200, "text/plain", "OK");
        } else if (inputMessage2 == COMMAND_TOGGLE) {
          serprln("toggle: " + inputMessage + " -- " + inputMessage2);
          exec_command[inputMessage.toInt()]=3;
          request->send(200, "text/plain", "OK");
        } else if (inputMessage2 == "angle") {
          exec_command[inputMessage.toInt()]=2;
          exec_parameter[inputMessage.toInt()]=request->getParam("angle")->value().toInt();
          serprln("angle: " + inputMessage + " -- " + exec_parameter[inputMessage.toInt()]);
          request->send(200, "text/plain", "OK");
        } else {        
          serprln("ERR: " + inputMessage + " -- " + inputMessage2);
          request->send(500, "text/plain", "Wrong actuator number");          
        }
      } else {
        serprln("CMD: " + inputMessage + " -- " + inputMessage2);
        request->send(500, "text/plain", "Command Error");
      }
    });
  }

  // /apply_config - Apply configuration parameters
  server.on("/restart_device", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!require_auth(request)) return;
    serprln("HTTP: RESTART");

    config_save = true;
    esp_restart = true;
    http_redirect(request, "/", "20", page_restart_device);
    //http_redirect(request, "/", "20", "Configuration applied. Redirecting...");
  });

  // /apply_config - Apply configuration parameters
  server.on("/device_defaults", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!require_auth(request)) return;
    serprln("HTTP: RESTART");

    erase_parameters();

    config_save = true;
    esp_restart = true;
    http_redirect(request, "/", "20", page_restart_device);
  });

  // /apply_config - Apply configuration parameters
  server.on("/apply_config", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!require_auth(request)) return;
    String  new_param;

    serprln("HTTP: Apply Config");

    if (request->hasParam(PARAM_SSID)) {
      new_param = request->getParam(PARAM_SSID)->value();
      if (new_param != ssid) {
        ssid = new_param;
        config_save = true;
        esp_restart = true;
        serprln("New - SSID: " + ssid);
      }
    }

    if (request->hasParam(PARAM_PASS)) {
      new_param = request->getParam(PARAM_PASS)->value();
      if (new_param != password) {
        password = new_param;
        config_save = true;
        esp_restart = true;
        serprln("New - PASS: " + password);
      }
    }

    if (request->hasParam(PARAM_RFPOWER)) {
      new_param = request->getParam(PARAM_RFPOWER)->value();
      if (new_param != rf_power) {
        rf_power = new_param;
        config_save = true;
        esp_restart = true;
        serprln("New - RF Power: " + rf_power);
      }
    }

    if (request->hasParam(PARAM_MAC)) {
      char *newMAC;
      newMAC = (char*)malloc(20);
      uint8_t parsedMAC[6];

      new_param = request->getParam(PARAM_MAC)->value();
      new_param.toCharArray(newMAC, 20);
      newMAC[19]=0;

      sscanf(newMAC, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx", &parsedMAC[0], &parsedMAC[1], &parsedMAC[2], &parsedMAC[3], &parsedMAC[4], &parsedMAC[5]);

      if (parsedMAC[0] != MAC[0] || parsedMAC[1] != MAC[1] || parsedMAC[2] != MAC[2] || parsedMAC[3] != MAC[3] || parsedMAC[4] != MAC[4] || parsedMAC[5] != MAC[5]) {
        for (int ii=0; ii<6; ii++)
          MAC[ii]=parsedMAC[ii];
        config_save = true;
        esp_restart = true;
        serprln("New - MAC: " + macToString(MAC));
      }
      free(newMAC);
    }

    if (request->hasParam(PARAM_INPUT_DEVICE_TYPE)) {
      new_param = request->getParam(PARAM_INPUT_DEVICE_TYPE)->value();
      if (new_param != config_device_type) {
        config_device_type = new_param;
        config_save = true;
        esp_restart = true;
        serprln("New - Device type: " + config_device_type);
      }
    }

    if (request->hasParam(PARAM_INPUT_SHOW_DESCRIPTION)) {
      new_param = request->getParam(PARAM_INPUT_SHOW_DESCRIPTION)->value();
      if (new_param != config_show_description) {
        config_show_description = new_param;
        config_save = true;
        serprln("New - Show_description: " + config_show_description);
      }
    }

    if (request->hasParam(PARAM_INPUT_DEBUG_ENABLED)) {
      new_param = request->getParam(PARAM_INPUT_DEBUG_ENABLED)->value();
      bool new_debug_enabled = (new_param == "1");
      if (new_debug_enabled != debug_enabled) {
        debug_enabled = new_debug_enabled;
        config_save = true;
        Serial.println("New - Debug enabled: " + String(debug_enabled ? "1" : "0"));
      }
    }

    if (request->hasParam(PARAM_INPUT_6)) {
      new_param = request->getParam(PARAM_INPUT_6)->value();
      if (new_param != config_gpio_used_number) {
        int clamped = new_param.toInt();
        if (clamped < 0) clamped = 0;
        if (clamped > MAX_GPIOS) clamped = MAX_GPIOS;
        config_gpio_used_number = String(clamped);
        config_save = true;
        esp_restart = true;
        serprln("New - Number of outputs: " + config_gpio_used_number);
      }
    }

    if (request->hasParam(PARAM_GPIO_EXPANDER_TYPE)) {
      new_param = request->getParam(PARAM_GPIO_EXPANDER_TYPE)->value();
      if (new_param != config_gpio_use_expander) {
        config_gpio_use_expander = new_param;
        config_save = true;
        esp_restart = true;
        serprln("New - GPIO expander: " + config_gpio_use_expander);
        if (config_gpio_use_expander==GPIO_EXPANDER_PCF8575) {
          config_gpio_used_number="16";
        } else {
          config_gpio_used_number="8";
        }
      }
    }

    String test_param = "";
    for (int i=0; i<config_gpio_used_number.toInt() && i<MAX_GPIOS; i++) {
      test_param = String(PARAM_INPUT_GPIO) + String(i);
      if (request->hasParam(test_param)) {
        new_param = request->getParam(test_param)->value();
        //serprln("Has GPIO param " + String(test_param) + " = " + String(new_param));
        if (config_gpio[i] != new_param && (config_gpio_use_expander!="0" || (config_gpio_use_expander=="0" && new_param != "1" && new_param != "3" && new_param != "7"))) {
          config_gpio[i] = new_param;
          config_save = true;
          serpr("New - Output GPIO ");
          serpr(i);
          serpr(" GPIO: ");
          serprln(new_param);
        }
      }

      int found=0;
      uint32_t new_gpio_cap = 0;
      for (int j=0; j<=27; j++) {
        test_param = String(PARAM_INPUT_GPIO_CAPABILITY) + String(i) + "_" + String(j);
        if (request->hasParam(test_param)) {
          found++;
          new_param = request->getParam(test_param)->value();
          //serprln("Has GPIO cap param " + String(test_param) + " = " + new_param + " , found = " + String(found));
          if (new_param == "on") {
            new_gpio_cap |= GPIO_BIT(j);
          }
        }
      }
      //serprln("gpiocap = " + String(new_gpio_cap));
      if (config_gpio_cap[i] != new_gpio_cap && found>0) {
          config_gpio_cap[i] = new_gpio_cap;
          config_save = true;
          serpr("New - Output GPIO cap " + String(i));
          serprln(" GPIO CAP: " + String(new_gpio_cap));
      }     

      if (config_device_type == DEVICE_TYPE_ACTUATOR) {
        test_param = PARAM_INPUT_MIN_ANGLE + String(i);
        if (request->hasParam(test_param)) {
          new_param = request->getParam(test_param)->value();
          if (min_angle[i] != new_param) {
            min_angle[i] = new_param;
            config_save = true;
            serpr("New - Minimum angle ");
            serpr(i);
            serpr(" angle: ");
            serprln(new_param);
          }
        }

        test_param = PARAM_INPUT_MAX_ANGLE + String(i);
        if (request->hasParam(test_param)) {
          new_param = request->getParam(test_param)->value();
          if (max_angle[i] != new_param) {
            max_angle[i] = new_param;
            config_save = true;
            serpr("New - Maximum angle ");
            serpr(i);
            serpr(" angle: ");
            serprln(new_param);
          }
        }
      }

      test_param = PARAM_INPUT_MANUFACTURER + String(i);
      if (request->hasParam(test_param)) {
        new_param = request->getParam(test_param)->value();
        if (config_manufacturer[i] != new_param) {
          config_manufacturer[i] = new_param;
          config_save = true;
          serpr("New - Manufacturer ");
          serpr(i);
          serpr(" : ");
          serprln(new_param);
        }
      }

      test_param = PARAM_INPUT_DESCRIPTION + String(i);
      if (request->hasParam(test_param)) {
        new_param = request->getParam(test_param)->value();
        if (config_description[i] != new_param) {
          config_description[i] = new_param;
          config_save = true;
          serpr("New - Description ");
          serpr(i);
          serpr(" : ");
          serprln(new_param);
        }
      }
    }

    if (esp_restart == false) {
      http_redirect(request, "/", "1", "Configuration applied. Redirecting...");
    } else {
      http_redirect(request, "/", "20", "Configuration applied. Redirecting...");
    }    
  });

  // /c4update - Present page where user can upload firmware
  server.on("/c4update", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: Update/Config page");
    request->send_P(200, "text/html", c4update_html, processor);
  });

  // /c4config_js - Present page where user can upload firmware
  server.on("/c4status.js", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: config.js");
    request->send_P(200, "text/javascript", c4status_js, processor_js);
  });

  // /c4config_js - Present page where user can upload firmware
  server.on("/c4config.js", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: config.js");
    request->send_P(200, "text/javascript", c4config_js, processor_js);
  });

  // /c4clock_js - Present page where user can upload firmware
  server.on("/c4clock.js", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: c4_clock.js");
    request->send_P(200, "text/javascript", c4clock_js);
  });

  // /c4refreh_gpio_js - Present page where user can upload firmware
  server.on("/c4refresh_gpio.js", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: c4_refresh_gpio.js");
    request->send_P(200, "text/javascript", c4refresh_gpio_js);
  });

  // /c4refreh_gpio_js - Present page where user can upload firmware
  server.on("/c4get_gpio.txt", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: c4_get_gpio.js");
    request->send_P(200, "text/plain", c4get_gpio, processor);
  });

  // /c4config_js - Present page where user can upload firmware
  server.on("/c4config.css", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: config.css");
    request->send_P(200, "text/css", c4config_css);
  });

  // /c4config_js - Present page where user can upload firmware
  server.on("/index.css", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: index.css");
    request->send_P(200, "text/css", index_css);
  });

  // /c4logo.svg - Provide C4 logo
  server.on("/c4logo.svg", HTTP_GET, [](AsyncWebServerRequest *request){
    //serprln("HTTP: c4logo.svg");
    request->send_P(200, "image/svg+xml", c4logo_svg);
  });

  // /doUpdate - Handle the OTA firmware upgrade
  server.on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      if (!request->authenticate("admin", password.c_str())) return request->requestAuthentication();
    },
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {
      if (!request->authenticate("admin", password.c_str())) return;
      handleDoUpdate(request, filename, index, data, len, final);
    }
  );

  // On error, print this page
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});

  // Start the MDNS service, after all web services are started
  MDNS.addService("http", "tcp", 80);

}
