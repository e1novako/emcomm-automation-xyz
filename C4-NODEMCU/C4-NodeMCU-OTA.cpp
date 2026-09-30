#define C4NODEMCU_OTA
#include "C4-NodeMCU.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/OtaUpload.h"

// Required for the OTA
#include <Updater.h>
#include <ESP8266mDNS.h>
#define U_PART U_FS
namespace {
struct FirmwareUpdateBackend {
  size_t contentLength = 0;
  int command = U_FLASH;

  bool begin() {
    Update.runAsync(true);
    return Update.begin(contentLength, command);
  }
  size_t write(const uint8_t *data, size_t length) {
    return Update.write(data, length);
  }
  bool finish() { return Update.end(true); }
  void abort() { Update.end(false); }
};

FirmwareUpdateBackend firmwareUpdateBackend;
emcomm::OtaUpload<FirmwareUpdateBackend> firmwareUpdate;
}

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
  if (len > 0 && progress != (prog*100)/len)
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

    firmwareUpdateBackend.contentLength = request->contentLength();
    firmwareUpdateBackend.command =
        (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;
    if (firmwareUpdate.begin(firmwareUpdateBackend) !=
        emcomm::OtaUploadState::Receiving) {
      Update.printError(Serial);
    }
  }

  if (firmwareUpdate.state() == emcomm::OtaUploadState::Receiving &&
      firmwareUpdate.write(firmwareUpdateBackend, data, len) ==
          emcomm::OtaUploadState::Receiving) {
    printProgress(Update.progress(), Update.size());
  } else if (firmwareUpdate.state() == emcomm::OtaUploadState::WriteFailed) {
    Update.printError(Serial);
  }

  if (final && firmwareUpdate.state() == emcomm::OtaUploadState::Receiving) {
    if (firmwareUpdate.finish(firmwareUpdateBackend) !=
        emcomm::OtaUploadState::Complete) {
      Update.printError(Serial);
      request->send(500, "text/plain", "OTA update failed");
    } else {
      factory_default = true;
      http_redirect(request, "/", "20", page_restart_device);
      serprln("Update complete");
      esp_restart=true;
    }
  } else if (final && firmwareUpdate.state() != emcomm::OtaUploadState::Complete) {
    request->send(500, "text/plain", "OTA update failed");
  }
}
