#include "Actions.h"
#include "Config.h"
#include "Debug.h"
#include "MqttClient.h"
#include "Outputs.h"
#include "Reservations.h"
#include <Arduino.h>
extern "C" {
#include "user_interface.h"
}

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
static int8_t buttonOutputs[3];
static uint8_t buttonCounts[3];
static int8_t buttonContacts[2];
static uint8_t buttonContactCount = 0;
// Button pulses are timed by an os_timer instead of loop() polling so web/MQTT
// handling cannot stretch a pulse. The tick only touches GPIOs and these
// flags; completion/logging is done from maintainBackgroundAction().
static os_timer_t buttonTimer;
static bool buttonTimerReady = false;
static volatile bool buttonPressed = false, buttonSeqDone = false;
static volatile uint8_t buttonRemaining = 0;
static uint16_t buttonPulseMs = 200, buttonPauseMs = 200;
// Tick timestamps (ms since sequence start) so real pulse/pause durations can
// be reported under debug logging.
constexpr uint8_t BUTTON_EVENT_MAX = 64;
static uint16_t buttonEvents[BUTTON_EVENT_MAX];
static uint8_t buttonEventCount = 0;
static volatile int8_t buttonPressLevel[3] = {-1, -1, -1};
// Level read just before release: LOW means nothing overwrote the pulse.
static volatile int8_t buttonHoldLevel[3] = {-1, -1, -1};
static unsigned long buttonSeqStartMs = 0;

static void recordButtonEvent() {
  if (buttonEventCount < BUTTON_EVENT_MAX)
    buttonEvents[buttonEventCount++] =
        static_cast<uint16_t>(millis() - buttonSeqStartMs);
}

static void buttonTimerTick(void *) {
  recordButtonEvent();
  if (buttonPressed) {
    if (buttonStage < 3)
      buttonHoldLevel[buttonStage] = digitalRead(buttonOutputs[buttonStage]);
    setButtonPin(buttonOutputs[buttonStage], false);
    buttonPressed = false;
    if (buttonRemaining == 0) {
      if (++buttonStage >= buttonStageCount) {
        buttonSeqDone = true;
        return;
      }
      buttonRemaining = buttonCounts[buttonStage];
    }
    os_timer_arm(&buttonTimer, buttonPauseMs, false);
  } else {
    setButtonPin(buttonOutputs[buttonStage], true);
    if (buttonStage < 3 && buttonRemaining == buttonCounts[buttonStage])
      buttonPressLevel[buttonStage] = digitalRead(buttonOutputs[buttonStage]);
    --buttonRemaining;
    buttonPressed = true;
    os_timer_arm(&buttonTimer, buttonPulseMs, false);
  }
}

static void stopButtonTimer() {
  if (buttonTimerReady)
    os_timer_disarm(&buttonTimer);
  buttonPressed = false;
}
static SequenceProfile buttonProfile = {};
static uint16_t pendingOutputs = 0;
static const char *bulkCommand = nullptr;

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
  for (uint8_t i = 0; i < cfg.numOutputs; ++i)
    if (isManagedOutput(i)) {
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
  return buttonStageCount && bgAction.deviceIdx == idx;
}

bool startButtonAction(uint8_t idx, uint8_t top, uint8_t bottom,
                       const char *name) {
  if (isActionRunning() || idx >= cfg.numOutputs || !outputsActivated)
    return false;
  int8_t topPin = cfg.devices[idx].topPin;
  int8_t bottomPin = cfg.devices[idx].bottomPin;
  if ((top && !isValidOutputPin(topPin)) ||
      (bottom && !isValidOutputPin(bottomPin)) ||
      (top && bottom && topPin == bottomPin))
    return false;
  buttonContactCount = 0;
  if (isValidOutputPin(topPin))
    buttonContacts[buttonContactCount++] = topPin;
  if (isValidOutputPin(bottomPin))
    buttonContacts[buttonContactCount++] = bottomPin;
  buttonStage = 0;
  buttonStageCount = top && bottom ? 3 : 1;
  buttonOutputs[0] = top ? topPin : bottomPin;
  buttonCounts[0] = top ? top : bottom;
  if (buttonStageCount == 3) {
    buttonOutputs[1] = bottomPin;
    buttonCounts[1] = bottom;
    buttonOutputs[2] = topPin;
    buttonCounts[2] = top;
  }
  buttonProfile.name = name;
  buttonPulseMs = cfg.buttonPulseMs;
  buttonPauseMs = cfg.buttonPauseMs;
  bgAction = {APHASE_BUTTON_RELEASE, &buttonProfile, idx, buttonCounts[0],
              millis(), false};
  // Release both contacts, then start the timer-driven pulse train: after one
  // pause the first press begins.
  for (uint8_t i = 0; i < buttonContactCount; ++i)
    setButtonPin(buttonContacts[i], false);
  if (!buttonTimerReady) {
    os_timer_setfn(&buttonTimer, buttonTimerTick, nullptr);
    buttonTimerReady = true;
  }
  buttonRemaining = buttonCounts[0];
  buttonPressed = false;
  buttonSeqDone = false;
  buttonEventCount = 0;
  buttonSeqStartMs = millis();
  os_timer_arm(&buttonTimer, buttonPauseMs, false);
  DBG("[MODEL] output=%u action=%s topGPIO=%d bottomGPIO=%d", idx + 1, name,
      topPin, bottomPin);
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

static void reportButtonTiming() {
  // Even-numbered events end a pause (press starts), odd end a pulse.
  unsigned minPulse = 65535, maxPulse = 0, minPause = 65535, maxPause = 0;
  for (uint8_t i = 1; i < buttonEventCount; ++i) {
    unsigned d = buttonEvents[i] - buttonEvents[i - 1];
    if (i % 2) {
      minPulse = d < minPulse ? d : minPulse;
      maxPulse = d > maxPulse ? d : maxPulse;
    } else {
      minPause = d < minPause ? d : minPause;
      maxPause = d > maxPause ? d : maxPause;
    }
  }
  // Count presses per GPIO (odd events are releases of a press).
  for (uint8_t s = 0; s < buttonStageCount; ++s)
    DBG("[MODEL] stage %u: GPIO %d x%u (pad level at press %d, before release %d)", s,
        buttonOutputs[s], buttonCounts[s], buttonPressLevel[s],
        buttonHoldLevel[s]);
  DBG("[MODEL] %u ticks; pulse %u-%u ms (cfg %u), pause %u-%u ms (cfg %u)",
      buttonEventCount, minPulse, maxPulse, buttonPulseMs, minPause, maxPause,
      buttonPauseMs);
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
  if (buttonStageCount)
    reportButtonTiming();
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
    if (buttonSeqDone) {
      stopButtonTimer();
      for (uint8_t i = 0; i < buttonContactCount; ++i)
        setButtonPin(buttonContacts[i], false);
      finishAction();
    }
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
  stopButtonTimer();
  for (uint8_t i = 0; i < buttonContactCount; ++i)
    setButtonPin(buttonContacts[i], false);
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

static bool buttonChannel(uint8_t idx) {
  return startButtonAction(idx, 7, 0, "channel");
}

bool handleLoadAction(uint8_t idx, const String &cmd) {
  if (idx >= cfg.numOutputs || !isManagedOutput(idx))
    return false;
  const DeviceModel &model = getDeviceModel(cfg.devices[idx].deviceModel);
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
  else if (model.usesButtons && cmd == F("channel"))
    action = buttonChannel;
  else if (model.usesButtons && cmd == F("button_top"))
    action = model.turn_on;
  else if (model.usesButtons && cmd == F("button_bottom"))
    action = model.turn_off;
  DBG("[MODEL] output=%u model=%s command=%s", idx + 1, model.name, cmd.c_str());
  return action && action(idx);
}

} // namespace vibrant
