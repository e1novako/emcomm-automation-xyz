#define C4NODEMCU_OTA
#include "C4-NodeMCU.h"

// Required for the OTA
#include <Updater.h>
#include <ESP8266mDNS.h>
#define U_PART U_FS
size_t content_len;

const char page_restart_device[] PROGMEM = R"rawliteral(
  <head></head>
  <body>
    Please wait while the device restarts...
    <script>
      sleep(10000);
      location.href="/";
    </script>
  </body>
)rawliteral";

// Print the OTA progress on serial port
void printProgress(int prog, int len) {
  if (progress != (prog*100)/len)
    serprf("Progress: %d%%\n", progress = (prog*100)/len);
}

// Redirect to another page
void http_redirect(AsyncWebServerRequest *request, String location, String refresh_time, String msg) {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", msg);
    response->addHeader("Refresh", refresh_time);
    response->addHeader("Location", location);
    request->send(response);
}

// /doUpdate - Accept and process update
void handleDoUpdate(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    serprln("Update - BEGIN ...");

    // Stop filesystem before the update
    //LittleFS.end();

    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;
    Update.runAsync(true);
    if (!Update.begin(content_len, cmd)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  } else {
    printProgress(Update.progress(), Update.size());
  }

  if (final) {
    factory_default = true;
    http_redirect(request, "/", "20", page_restart_device);

    if (!Update.end(true)) {
      Update.printError(Serial);
      //LittleFS.begin();
    } else {
      serprln("Update complete");
      esp_restart=true;
    }
  }
}

