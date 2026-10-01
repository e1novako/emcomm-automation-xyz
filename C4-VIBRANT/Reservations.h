#pragma once
#include "Config.h"
namespace vibrant {
struct OutputReservation {
  bool reserved;
  String owner;
};
extern OutputReservation outputReservations[MAX_DEVICES];
void clearOutputReservations();
uint8_t managedOutputCount();
uint8_t availableManagedOutputCount();
} // namespace vibrant
