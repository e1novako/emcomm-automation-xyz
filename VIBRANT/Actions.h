#pragma once
#include "Config.h"
namespace vibrant {
struct SequenceProfile {
  const char *name;
  uint8_t cycles;
  unsigned long prepOnMs, cycleOffMs, cycleOnMs, finalWaitMs, triggerOffMs,
      triggerOnMs;
};
extern const SequenceProfile PROFILE_REBOOT, PROFILE_LEAVE_MESH,
    PROFILE_FACTORY_RESET;
enum ActionPhase : uint8_t {
  APHASE_NONE = 0,
  APHASE_PREP_ON,
  APHASE_CYCLE_OFF,
  APHASE_CYCLE_ON,
  APHASE_FINAL_WAIT,
  APHASE_TRIGGER_OFF,
  APHASE_TRIGGER_ON
};
struct ActiveAction {
  ActionPhase phase;
  const SequenceProfile *profile;
  uint8_t deviceIdx, cyclesRemaining;
  unsigned long phaseStartMs;
  bool allOutputs;
};
extern ActiveAction bgAction;
bool isActionRunning();
bool isInCyclePhase();
String actionPhaseName();
void setPhaseOutputState(bool);
void enterPhase(ActionPhase, bool);
void startSequence(const SequenceProfile &, uint8_t, bool);
void finishAction();
void maintainBackgroundAction();
void cancelAction();
bool handleLoadAction(uint8_t, const String &);
} // namespace vibrant
