#include "Reservations.h"
#include "Config.h"
#include "Outputs.h"

namespace vibrant {

OutputReservation outputReservations[MAX_DEVICES];

void clearOutputReservations() {
  for (uint8_t i = 0; i < MAX_DEVICES; ++i) {
    outputReservations[i].reserved = false;
    outputReservations[i].owner = "";
  }
}

uint8_t managedOutputCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i))
      ++count;
  }
  return count;
}

uint8_t availableManagedOutputCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < cfg.numOutputs; ++i) {
    if (isManagedOutput(i) && !outputReservations[i].reserved)
      ++count;
  }
  return count;
}

} // namespace vibrant
