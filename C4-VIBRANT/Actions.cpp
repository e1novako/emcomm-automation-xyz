#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"
#include "Reservations.h"
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
static uint8_t buttonStage = 0, buttonStageCount = 0;
static uint8_t buttonOutputs[3], buttonCounts[3];
static uint8_t buttonContacts[2], buttonContactCount = 0;
static SequenceProfile buttonProfile = {};
static uint16_t pendingOutputs = 0;
static const char *bulkCommand = nullptr;

bool isButtonHelperOutput(uint8_t idx) {
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (i == idx || !getDeviceModel(cfg.devices[i].deviceModel).usesButtons)
      continue;
    if (cfg.devices[i].topOutput == idx || cfg.devices[i].bottomOutput == idx)
      return true;
  }
  return false;
}

static void startNextModelAction() {
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (!(pendingOutputs & (1U << i)))
      continue;
    pendingOutputs &= ~(1U << i);
    if (handleLoadAction(i, String(bulkCommand))) {
      if (isActionRunning())
        return;
    } else {
      logWarning(String(F("Bulk action skipped invalid button mapping: output ")) +
                 String(i + 1));
    }
  }
}

bool startModelActionAll(const char *command) {
  if (isActionRunning())
    return false;
  bulkCommand = command;
  pendingOutputs = 0;
  bool bulbsOnly = true;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i)
    if (getDeviceModel(cfg.devices[i].deviceModel).usesButtons)
      bulbsOnly = false;
    else if (isManagedOutput(i) && isButtonHelperOutput(i))
      bulbsOnly = false;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i)
    if (isManagedOutput(i) && !isButtonHelperOutput(i)) {
      pendingOutputs |= 1U << i;
      bulbsOnly &= !getDeviceModel(cfg.devices[i].deviceModel).usesButtons;
    }
  if (pendingOutputs && bulbsOnly &&
      (String(command) == F("leave_mesh") || String(command) == F("factory_reset"))) {
    pendingOutputs = 0;
    startSequence(String(command) == F("factory_reset") ? PROFILE_FACTORY_RESET
                                                      : PROFILE_LEAVE_MESH,
                  0, true);
    return true;
  }
  startNextModelAction();
  return isActionRunning();
}

bool buttonActionOwnsOutput(uint8_t idx) {
  if (pendingOutputs)
    return isManagedOutput(idx);
  for (uint8_t i = 0; i < buttonContactCount; ++i)
    if (buttonContacts[i] == idx)
      return true;
  return false;
}

bool startButtonAction(uint8_t idx, uint8_t top, uint8_t bottom,
                       const char *name) {
  if (isActionRunning() || idx >= cfg.numOutputs || !outputsActivated)
    return false;
  int topIdx = cfg.devices[idx].topOutput;
  int bottomIdx = cfg.devices[idx].bottomOutput;
  auto valid = [idx](int output) {
    return output >= 0 && output < cfg.numOutputs &&
           isManagedOutput(output) &&
           (output == idx || !outputReservations[output].reserved);
  };
  if ((top && !valid(topIdx)) || (bottom && !valid(bottomIdx)) ||
      (topIdx >= 0 && !valid(topIdx)) ||
      (bottomIdx >= 0 && !valid(bottomIdx)) ||
      (top && bottom && cfg.devices[topIdx].pin == cfg.devices[bottomIdx].pin))
    return false;
  if (topIdx >= 0 && bottomIdx >= 0 &&
      cfg.devices[topIdx].pin == cfg.devices[bottomIdx].pin)
    return false;
  buttonContactCount = 0;
  if (topIdx >= 0)
    buttonContacts[buttonContactCount++] = topIdx;
  if (bottomIdx >= 0)
    buttonContacts[buttonContactCount++] = bottomIdx;
  buttonStage = 0;
  buttonStageCount = top && bottom ? 3 : 1;
  buttonOutputs[0] = top ? topIdx : bottomIdx;
  buttonCounts[0] = top ? top : bottom;
  if (buttonStageCount == 3) {
    buttonOutputs[1] = bottomIdx;
    buttonCounts[1] = bottom;
    buttonOutputs[2] = topIdx;
    buttonCounts[2] = top;
  }
  buttonProfile.name = name;
  bgAction = {APHASE_BUTTON_RELEASE, &buttonProfile, idx, buttonCounts[0],
              millis(), false};
  // Release both contacts before the legacy 800 ms preparation interval.
  for (uint8_t i = 0; i < buttonContactCount; ++i) {
    setOutputDirect(buttonContacts[i], false);
    mqttPublishOutputState(buttonContacts[i]);
  }
  DBG("[MODEL] output=%u action=%s top=%d bottom=%d", idx + 1, name,
      topIdx + 1, bottomIdx + 1);
  return true;
}

bool isActionRunning() { return bgAction.phase != APHASE_NONE; }

bool isInCyclePhase() {
  return bgAction.phase == APHASE_CYCLE_OFF ||
         bgAction.phase == APHASE_CYCLE_ON ||
         bgAction.phase == APHASE_BUTTON_PRESS ||
         bgAction.phase == APHASE_BUTTON_RELEASE;
}

String actionPhaseName() {
  if (buttonStageCount)
    return String(buttonProfile.name) +
           (bgAction.phase == APHASE_BUTTON_PRESS ? F(" pressing") : F(" released"));
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
  } else if (!buttonStageCount) {
    // Leave output ON after completing the sequence
    setOutputDirect(idx, true);
    mqttPublishOutputState(idx);
    logStatus(String(F("Load action complete for output ")) + String(idx + 1));
  } else {
    logStatus(String(F("Button action complete for output ")) + String(idx + 1));
  }
  buttonStageCount = 0;
  buttonContactCount = 0;
  startNextModelAction();
}

void maintainBackgroundAction() {
  if (!isActionRunning())
    return;
  const SequenceProfile &profile = *bgAction.profile;
  unsigned long elapsed = millis() - bgAction.phaseStartMs;
  if (buttonStageCount) {
    unsigned long wait = bgAction.phase == APHASE_BUTTON_RELEASE &&
                                 bgAction.cyclesRemaining == buttonCounts[buttonStage]
                             ? 800UL
                             : 150UL;
    if (elapsed < wait)
      return;
    uint8_t output = buttonOutputs[buttonStage];
    if (bgAction.phase == APHASE_BUTTON_PRESS) {
      setOutputDirect(output, false);
      bgAction.phase = APHASE_BUTTON_RELEASE;
    } else {
      if (bgAction.cyclesRemaining == 0) {
        if (++buttonStage == buttonStageCount) {
          finishAction();
          return;
        }
        bgAction.cyclesRemaining = buttonCounts[buttonStage];
        output = buttonOutputs[buttonStage];
      }
      setOutputDirect(output, true);
      --bgAction.cyclesRemaining;
      bgAction.phase = APHASE_BUTTON_PRESS;
    }
    mqttPublishOutputState(output);
    bgAction.phaseStartMs = millis();
    DBG("[MODEL] %s output=%u remaining=%u", actionPhaseName().c_str(),
        output + 1, bgAction.cyclesRemaining);
    return;
  }
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
  for (uint8_t i = 0; i < buttonContactCount; ++i) {
    setOutputDirect(buttonContacts[i], false);
    mqttPublishOutputState(buttonContacts[i]);
  }
  buttonStageCount = 0;
  buttonContactCount = 0;
  pendingOutputs = 0;
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
  if (idx >= cfg.numOutputs || !isValidOutputPin(cfg.devices[idx].pin))
    return false;
  const DeviceModel &model = getDeviceModel(cfg.devices[idx].deviceModel);
  // Contact-only rows must never latch a button closed via raw power commands.
  if (!model.usesButtons && isButtonHelperOutput(idx))
    return false;
  bool (*action)(uint8_t) = nullptr;
  if (cmd == F("power_on") || cmd == F("turn_on") || cmd == F("on"))
    action = model.turn_on;
  else if (cmd == F("power_off") || cmd == F("turn_off") || cmd == F("off"))
    action = model.turn_off;
  else if (cmd == F("toggle"))
    action = model.toggle;
  else if (cmd == F("identify") || cmd == F("join") || cmd == F("join_network"))
    action = model.identify;
  else if (cmd == F("leave_mesh") || cmd == F("leave"))
    action = model.leave_mesh;
  else if (cmd == F("factory_reset"))
    action = model.factory_reset;
  else if (cmd == F("reboot"))
    action = model.reboot;
  else if (model.usesButtons && cmd == F("button_top"))
    action = model.turn_on;
  else if (model.usesButtons && cmd == F("button_bottom"))
    action = model.turn_off;
  DBG("[MODEL] output=%u model=%s command=%s", idx + 1, model.name, cmd.c_str());
  return action && action(idx);
}

} // namespace vibrant
