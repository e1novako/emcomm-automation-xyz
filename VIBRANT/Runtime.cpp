#include "Runtime.h"
#include "MqttClient.h"
#include "OtaService.h"
#include "Outputs.h"
#include "WifiManager.h"

namespace vibrant {

void applyRuntimeSettings() {
  applyWifiSettings();
  refreshOutputsForCurrentBootPhase();
  applyMqttSettings();
  mqttEnsureConnected();
  applyArduinoOtaSettings();
}

} // namespace vibrant
