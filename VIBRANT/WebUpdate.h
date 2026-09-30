#pragma once
#include <Arduino.h>
namespace vibrant {
extern bool otaUpdateFailed;
extern String otaUpdateError;
void handleFirmwareUpdatePage();
void handleFirmwareUpdateUpload();
void handleFirmwareUpdateDone();
} // namespace vibrant
