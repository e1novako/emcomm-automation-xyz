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

void setButtonPin(int8_t pin, bool pressed) {
  if (!isValidOutputPin(pin))
    return;
  pinMode(pin, OUTPUT);
  // Active-low like the other outputs: pressed -> LOW, released -> HIGH.
  digitalWrite(pin, pressed ? LOW : HIGH);
}

void applyOutputsNow() {
  logStatus(F("Applying output states..."));
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    if (getDeviceModel(cfg.devices[i].deviceModel).usesButtons) {
      // Button rows have no level output; hold both button contacts released.
      if (!isActionRunning() || i != bgAction.deviceIdx) {
        setButtonPin(cfg.devices[i].topPin, false);
        setButtonPin(cfg.devices[i].bottomPin, false);
      }
      continue;
    }
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
  if (outputsActivated && !getDeviceModel(d.deviceModel).usesButtons &&
      isValidOutputPin(d.pin)) {
    pinMode(d.pin, OUTPUT);
    // Inverted output logic: logical ON -> LOW, logical OFF -> HIGH
    // (active-low).
    digitalWrite(d.pin, state ? LOW : HIGH);
  }
}

bool isManagedOutput(uint8_t idx) {
  if (idx >= cfg.numOutputs)
    return false;
  const DeviceEntry &d = cfg.devices[idx];
  if (getDeviceModel(d.deviceModel).usesButtons)
    return isValidOutputPin(d.topPin) || isValidOutputPin(d.bottomPin);
  return isValidOutputPin(d.pin);
}

bool actionOwnsOutput(uint8_t idx) {
  return isActionRunning() &&
         (buttonActionOwnsOutput(idx) ||
          (bgAction.allOutputs ? isManagedOutput(idx)
                               : bgAction.deviceIdx == idx));
}

} // namespace vibrant
