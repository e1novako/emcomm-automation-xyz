#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"
#include <Arduino.h>

namespace vibrant {

constexpr uint8_t LEAVE_MESH_CYCLES = 5, REBOOT_SEQUENCE_CYCLES = 1;
constexpr unsigned long ACTION_CYCLE_OFF_MS = 5000UL,
                        ACTION_CYCLE_ON_MS = 1000UL,
                        ACTION_FINAL_WAIT_MS = 5000UL,
                        ACTION_TRIGGER_OFF_MS = 2000UL,
                        ACTION_TRIGGER_ON_MS = 1000UL;
constexpr unsigned long LEAVE_MESH_PREP_ON_MS = 8000UL,
                        LEAVE_MESH_CYCLE_OFF_MS = 1000UL,
                        LEAVE_MESH_CYCLE_ON_MS = 1500UL,
                        LEAVE_MESH_FINAL_WAIT_MS = 8000UL,
                        LEAVE_MESH_TRIGGER_OFF_MS = 1000UL,
                        LEAVE_MESH_TRIGGER_ON_MS = 10000UL;
constexpr uint8_t FACTORY_RESET_CYCLES = 13;
constexpr unsigned long FACTORY_RESET_PREP_ON_MS = 8000UL,
                        FACTORY_RESET_CYCLE_OFF_MS = 1000UL,
                        FACTORY_RESET_CYCLE_ON_MS = 1500UL,
                        FACTORY_RESET_FINAL_WAIT_MS = 8000UL,
                        FACTORY_RESET_TRIGGER_OFF_MS = 1000UL,
                        FACTORY_RESET_TRIGGER_ON_MS = 10000UL;
const SequenceProfile PROFILE_REBOOT = {"reboot",
                                        REBOOT_SEQUENCE_CYCLES,
                                        0,
                                        ACTION_CYCLE_OFF_MS,
                                        ACTION_CYCLE_ON_MS,
                                        ACTION_FINAL_WAIT_MS,
                                        ACTION_TRIGGER_OFF_MS,
                                        ACTION_TRIGGER_ON_MS};
const SequenceProfile PROFILE_LEAVE_MESH = {"leave mesh",
                                            LEAVE_MESH_CYCLES,
                                            LEAVE_MESH_PREP_ON_MS,
                                            LEAVE_MESH_CYCLE_OFF_MS,
                                            LEAVE_MESH_CYCLE_ON_MS,
                                            LEAVE_MESH_FINAL_WAIT_MS,
                                            LEAVE_MESH_TRIGGER_OFF_MS,
                                            LEAVE_MESH_TRIGGER_ON_MS};
const SequenceProfile PROFILE_FACTORY_RESET = {"factory reset",
                                               FACTORY_RESET_CYCLES,
                                               FACTORY_RESET_PREP_ON_MS,
                                               FACTORY_RESET_CYCLE_OFF_MS,
                                               FACTORY_RESET_CYCLE_ON_MS,
                                               FACTORY_RESET_FINAL_WAIT_MS,
                                               FACTORY_RESET_TRIGGER_OFF_MS,
                                               FACTORY_RESET_TRIGGER_ON_MS};
ActiveAction bgAction = {APHASE_NONE, nullptr, 0, 0, 0UL, false};

bool isActionRunning() { return bgAction.phase != APHASE_NONE; }

bool isInCyclePhase() {
  return bgAction.phase == APHASE_CYCLE_OFF ||
         bgAction.phase == APHASE_CYCLE_ON;
}

String actionPhaseName() {
  String phase;
  switch (bgAction.phase) {
  case APHASE_PREP_ON:
    phase = F("prep on");
    break;
  case APHASE_CYCLE_OFF:
    phase = F("cycling off");
    break;
  case APHASE_CYCLE_ON:
    phase = F("cycling on");
    break;
  case APHASE_FINAL_WAIT:
    phase = F("waiting (final)");
    break;
  case APHASE_TRIGGER_OFF:
    phase = F("triggering off");
    break;
  case APHASE_TRIGGER_ON:
    phase = F("triggering on");
    break;
  default:
    return F("idle");
  }
  return String(bgAction.profile->name) + ' ' + phase;
}

void setPhaseOutputState(bool state) {
  if (bgAction.allOutputs) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i)) {
        setOutputDirect(i, state);
        mqttPublishOutputState(i);
      }
    }
  } else {
    setOutputDirect(bgAction.deviceIdx, state);
    mqttPublishOutputState(bgAction.deviceIdx);
  }
}

void enterPhase(ActionPhase phase, bool outputState) {
  setPhaseOutputState(outputState);
  bgAction.phase = phase;
  bgAction.phaseStartMs = millis();
  DBG("%s%s: %s, cycles remaining=%u", bgAction.profile->name,
      bgAction.allOutputs ? " all" : "", actionPhaseName().c_str(),
      static_cast<unsigned>(bgAction.cyclesRemaining));
}

void startSequence(const SequenceProfile &profile, uint8_t idx,
                   bool allOutputs) {
  bgAction.profile = &profile;
  bgAction.deviceIdx = idx;
  bgAction.allOutputs = allOutputs;
  bgAction.cyclesRemaining = profile.cycles;
  if (profile.prepOnMs && (allOutputs || !cfg.devices[idx].state)) {
    enterPhase(APHASE_PREP_ON, true);
  } else {
    enterPhase(APHASE_CYCLE_OFF, false);
  }
}

void finishAction() {
  uint8_t idx = bgAction.deviceIdx;
  bool wasAll = bgAction.allOutputs;
  bgAction.phase = APHASE_NONE;
  bgAction.profile = nullptr;
  bgAction.allOutputs = false;
  if (wasAll) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i)) {
        setOutputDirect(i, true);
        mqttPublishOutputState(i);
      }
    }
    logStatus(F("Load action complete for all outputs"));
  } else {
    // Leave output ON after completing the sequence
    setOutputDirect(idx, true);
    mqttPublishOutputState(idx);
    logStatus(String(F("Load action complete for output ")) + String(idx + 1));
  }
}

void maintainBackgroundAction() {
  if (!isActionRunning())
    return;
  const SequenceProfile &profile = *bgAction.profile;
  unsigned long elapsed = millis() - bgAction.phaseStartMs;
  switch (bgAction.phase) {
  case APHASE_PREP_ON:
    if (elapsed >= profile.prepOnMs)
      enterPhase(APHASE_CYCLE_OFF, false);
    break;
  case APHASE_CYCLE_OFF:
    if (elapsed >= profile.cycleOffMs)
      enterPhase(APHASE_CYCLE_ON, true);
    break;
  case APHASE_CYCLE_ON:
    if (elapsed >= profile.cycleOnMs) {
      if (--bgAction.cyclesRemaining > 0) {
        enterPhase(APHASE_CYCLE_OFF, false);
      } else {
        bgAction.phase = APHASE_FINAL_WAIT;
        bgAction.phaseStartMs = millis();
        DBG("%s", actionPhaseName().c_str());
      }
    }
    break;
  case APHASE_FINAL_WAIT:
    if (elapsed >= profile.finalWaitMs)
      enterPhase(APHASE_TRIGGER_OFF, false);
    break;
  case APHASE_TRIGGER_OFF:
    if (elapsed >= profile.triggerOffMs)
      enterPhase(APHASE_TRIGGER_ON, true);
    break;
  case APHASE_TRIGGER_ON:
    if (elapsed >= profile.triggerOnMs)
      finishAction();
    break;
  default:
    bgAction.phase = APHASE_NONE;
    bgAction.profile = nullptr;
    break;
  }
}

void cancelAction() {
  if (!isActionRunning())
    return;
  uint8_t idx = bgAction.deviceIdx;
  bool wasAll = bgAction.allOutputs;
  bgAction.phase = APHASE_NONE;
  bgAction.profile = nullptr;
  bgAction.allOutputs = false;
  if (wasAll) {
    for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
      if (isManagedOutput(i))
        mqttPublishOutputState(i);
    }
    logStatus(F("Action cancelled for all outputs"));
  } else {
    logStatus(String(F("Action cancelled for output ")) + String(idx + 1));
    mqttPublishOutputState(idx);
  }
}

bool handleLoadAction(uint8_t idx, const String &cmd) {
  if (idx >= cfg.numOutputs)
    return false;
  if (cmd == F("power_on")) {
    if (actionOwnsOutput(idx))
      cancelAction();
    setOutputDirect(idx, true);
    mqttPublishOutputState(idx);
    maybePersistOutputState();
    return true;
  }
  if (cmd == F("power_off")) {
    if (actionOwnsOutput(idx))
      cancelAction();
    setOutputDirect(idx, false);
    mqttPublishOutputState(idx);
    maybePersistOutputState();
    return true;
  }
  if (!isValidOutputPin(cfg.devices[idx].pin))
    return false;
  if (isActionRunning())
    return false;
  if (cmd == F("leave_mesh")) {
    logStatus(String(F("Starting leave_mesh action for output ")) +
              String(idx + 1));
    startSequence(PROFILE_LEAVE_MESH, idx, false);
    return true;
  }
  if (cmd == F("factory_reset")) {
    logStatus(String(F("Starting factory_reset action for output ")) +
              String(idx + 1));
    startSequence(PROFILE_FACTORY_RESET, idx, false);
    return true;
  }
  if (cmd == F("reboot")) {
    logStatus(String(F("Starting reboot action for output ")) +
              String(idx + 1));
    startSequence(PROFILE_REBOOT, idx, false);
    return true;
  }
  return false;
}

} // namespace vibrant
