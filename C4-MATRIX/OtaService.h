#pragma once
namespace c4matrix {
extern bool arduinoOtaActive;
extern bool otaTransferInProgress;
void applyArduinoOtaSettings();
void maintainArduinoOta();
}
