#pragma once
#include "Config.h"
namespace vibrant {
extern bool outputsActivated, outputActivationDeferredLogged;
extern unsigned long bootStartMillis;
const PinMapping *findPinMapping(int);
String pinLabel(int8_t);
bool isValidOutputPin(int8_t);
void logDeviceSummary();
void applyOutputsNow();
bool outputActivationDelayElapsed();
void logDeferredOutputActivation();
void applyOutputsWhenSafe();
void refreshOutputsForCurrentBootPhase();
void prepareOutputsForBootPhase();
void setOutputDirect(uint8_t, bool);
bool isManagedOutput(uint8_t);
bool actionOwnsOutput(uint8_t);
} // namespace vibrant
