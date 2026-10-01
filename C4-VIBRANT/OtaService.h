#pragma once
namespace vibrant {
extern bool arduinoOtaActive, arduinoOtaCallbacksConfigured;
// True only while an ArduinoOTA (network/IDE) firmware transfer is actively
// in flight (between onStart and onEnd/onError). loop() uses this to pause
// unrelated work -- web server, MQTT, stickserver discovery -- so a slow
// web request or MQTT burst can never starve ArduinoOTA.handle() of CPU
// time in the middle of writing flash, which was causing uploads to time
// out and require 2-3 retries.
extern bool otaTransferInProgress;
void applyArduinoOtaSettings();
} // namespace vibrant
