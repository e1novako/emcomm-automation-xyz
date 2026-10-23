#include "DeviceModels.h"
#include "Actions.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"

namespace vibrant {
static bool power(uint8_t idx, bool on) {
  if (actionOwnsOutput(idx))
    cancelAction();
  setOutputDirect(idx, on);
  mqttPublishOutputState(idx);
  maybePersistOutputState();
  return true;
}
static bool bulbOn(uint8_t idx) { return power(idx, true); }
static bool bulbOff(uint8_t idx) { return power(idx, false); }
static bool bulbToggle(uint8_t idx) {
  return power(idx, !cfg.devices[idx].state);
}
static bool bulbSequence(uint8_t idx, const SequenceProfile &profile) {
  if (isActionRunning())
    return false;
  startSequence(profile, idx, false);
  return true;
}
static bool bulbLeave(uint8_t idx) {
  return bulbSequence(idx, PROFILE_LEAVE_MESH);
}
static bool bulbReset(uint8_t idx) {
  return bulbSequence(idx, PROFILE_FACTORY_RESET);
}
static bool bulbReboot(uint8_t idx) {
  return bulbSequence(idx, PROFILE_REBOOT);
}
static bool buttonIdentify(uint8_t idx) {
  return startButtonAction(idx, 4, 0, "identify");
}
static bool buttonLeave(uint8_t idx) {
  return startButtonAction(idx, 13, 4, "leave mesh");
}
static bool buttonReset(uint8_t idx) {
  return startButtonAction(idx, 9, 4, "factory reset");
}
// cfg.devices[idx].state tracks the DUT power state implied by the last
// top (on) / bottom (off) press so Toggle alternates between the buttons.
static bool buttonTop(uint8_t idx) {
  bool ok = startButtonAction(idx, 1, 0, "top button");
  if (ok)
    cfg.devices[idx].state = true;
  return ok;
}
static bool buttonBottom(uint8_t idx) {
  bool ok = startButtonAction(idx, 0, 1, "bottom button");
  if (ok)
    cfg.devices[idx].state = false;
  return ok;
}
static bool buttonToggle(uint8_t idx) {
  bool on = cfg.devices[idx].state;
  DBG("[MODEL] output=%u toggle -> %s button", idx + 1, on ? "bottom" : "top");
  return on ? buttonBottom(idx) : buttonTop(idx);
}
static bool buttonReboot(uint8_t idx) {
  return startButtonAction(idx, 15, 0, "reboot");
}
const DeviceModel DEVICE_MODELS[] = {
    {MODEL_VIBRANT, "Control4 Vibrant", false, bulbOn, bulbLeave, bulbReset,
     bulbOn, bulbOff, bulbToggle, bulbReboot},
    {MODEL_CONTROL4_GEN3, "Control4 Gen3", true, buttonIdentify, buttonLeave,
     buttonReset, buttonTop, buttonBottom, buttonToggle, buttonReboot},
    {MODEL_CONTROL4_GEN4, "Control4 Gen4", true, buttonIdentify, buttonLeave,
     buttonReset, buttonTop, buttonBottom, buttonToggle, buttonReboot}};
const size_t DEVICE_MODEL_COUNT = sizeof(DEVICE_MODELS) / sizeof(DEVICE_MODELS[0]);
bool isValidDeviceModel(int type) {
  for (size_t i = 0; i < DEVICE_MODEL_COUNT; ++i)
    if (type == DEVICE_MODELS[i].type)
      return true;
  return false;
}
const DeviceModel &getDeviceModel(int type) {
  for (size_t i = 0; i < DEVICE_MODEL_COUNT; ++i)
    if (type == DEVICE_MODELS[i].type)
      return DEVICE_MODELS[i];
  return DEVICE_MODELS[0];
}
} // namespace vibrant
