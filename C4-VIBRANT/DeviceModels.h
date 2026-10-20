#pragma once
#include <Arduino.h>

namespace vibrant {
enum DeviceModelType : uint8_t {
  MODEL_VIBRANT = 0,
  MODEL_CONTROL4_GEN3 = 1,
  MODEL_CONTROL4_GEN4 = 2
};
struct DeviceModel {
  DeviceModelType type;
  const char *name;
  bool usesButtons;
  bool (*identify)(uint8_t);
  bool (*leave_mesh)(uint8_t);
  bool (*factory_reset)(uint8_t);
  bool (*turn_on)(uint8_t);
  bool (*turn_off)(uint8_t);
  bool (*toggle)(uint8_t);
  bool (*reboot)(uint8_t);
};
extern const DeviceModel DEVICE_MODELS[];
extern const size_t DEVICE_MODEL_COUNT;
bool isValidDeviceModel(int);
const DeviceModel &getDeviceModel(int);
} // namespace vibrant
