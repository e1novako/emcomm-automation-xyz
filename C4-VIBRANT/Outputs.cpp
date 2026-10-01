#include "Outputs.h"
#include "Actions.h"
#include "Debug.h"
#include <Arduino.h>

namespace vibrant {

bool outputsActivated = false;
bool outputActivationDeferredLogged = false;
unsigned long bootStartMillis = 0;

const PinMapping *findPinMapping(int pin) {
  for (size_t i = 0; i < OUTPUT_PIN_MAPPING_COUNT; ++i) {
    if (OUTPUT_PIN_MAPPINGS[i].gpio == pin) {
      return &OUTPUT_PIN_MAPPINGS[i];
    }
  }
  return nullptr;
}

String pinLabel(int8_t pin) {
  const PinMapping *mapping = findPinMapping(pin);
  return mapping ? String(mapping->label) : String(F("none"));
}

bool isValidOutputPin(int8_t pin) { return findPinMapping(pin) != nullptr; }

void applyOutputsNow() {
  logStatus(F("Applying output states..."));
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    int8_t pin = cfg.devices[i].pin;
    if (!isValidOutputPin(pin)) {
      continue;
    }
    pinMode(pin, OUTPUT);
    // Inverted output logic: logical ON -> LOW, logical OFF -> HIGH
    // (active-low).
    digitalWrite(pin, cfg.devices[i].state ? LOW : HIGH);
  }
  outputsActivated = true;
  outputActivationDeferredLogged = false;
  logDeviceSummary();
}

bool outputActivationDelayElapsed() {
  // Unsigned subtraction keeps this short post-boot elapsed-time check valid
  // even if millis() later wraps around.
  return (millis() - bootStartMillis) >= OUTPUT_BOOT_ACTIVATION_DELAY_MS;
}

void logDeferredOutputActivation() {
  if (outputActivationDeferredLogged)
    return;
  logStatus(String(F("Deferring output activation until ")) +
            String(OUTPUT_BOOT_ACTIVATION_DELAY_MS) + F(" ms after boot."));
  outputActivationDeferredLogged = true;
}

void applyOutputsWhenSafe() {
  if (outputsActivated)
    return;
  if (!outputActivationDelayElapsed()) {
    logDeferredOutputActivation();
    return;
  }
  logStatus(F("Post-boot output activation delay elapsed."));
  applyOutputsNow();
}

void refreshOutputsForCurrentBootPhase() {
  if (outputsActivated || outputActivationDelayElapsed()) {
    applyOutputsNow();
    return;
  }
  logStatus(F("Output configuration updated during boot delay; hardware "
              "activation remains deferred."));
  outputActivationDeferredLogged = false;
  logDeferredOutputActivation();
}

void prepareOutputsForBootPhase() {
  if (outputsActivated)
    return;
  if (outputActivationDelayElapsed()) {
    applyOutputsNow();
    return;
  }
  logDeferredOutputActivation();
}

void setOutputDirect(uint8_t idx, bool state) {
  if (idx >= MAX_DEVICES)
    return;
  DeviceEntry &d = cfg.devices[idx];
  d.state = state;
  if (outputsActivated && isValidOutputPin(d.pin)) {
    pinMode(d.pin, OUTPUT);
    // Inverted output logic: logical ON -> LOW, logical OFF -> HIGH
    // (active-low).
    digitalWrite(d.pin, state ? LOW : HIGH);
  }
}

bool isManagedOutput(uint8_t idx) {
  return idx < cfg.numOutputs && isValidOutputPin(cfg.devices[idx].pin);
}

bool actionOwnsOutput(uint8_t idx) {
  return isActionRunning() && (bgAction.allOutputs ? isManagedOutput(idx)
                                                   : bgAction.deviceIdx == idx);
}

} // namespace vibrant
