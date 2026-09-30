#include "../libraries/EmcommCommon/src/EmcommCommon/Diagnostics.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/MacAddress.h"

#include <assert.h>
#include <stdint.h>

int main() {
  uint8_t address[6] = {0};
  assert(emcomm::parseMacAddress("00:1a:2B:3c:4D:5e", address));
  const uint8_t expected[6] = {0x00, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e};
  for (uint8_t i = 0; i < 6; ++i)
    assert(address[i] == expected[i]);

  const char *invalidAddresses[] = {
      nullptr,
      "",
      "00:1a:2B:3c:4D",
      "00:1a:2B:3c:4D:5e:6f",
      "00-1a-2B-3c-4D-5e",
      "00:1a:2B:3c:4D:5g",
      "0:1a:2B:3c:4D:5e",
      "00:1a:2B:3c:4D:5e ",
  };
  for (const char *invalid : invalidAddresses) {
    uint8_t unchanged[6] = {1, 2, 3, 4, 5, 6};
    assert(!emcomm::parseMacAddress(invalid, unchanged));
    for (uint8_t i = 0; i < 6; ++i)
      assert(unchanged[i] == i + 1);
  }
  assert(!emcomm::parseMacAddress("00:1a:2B:3c:4D:5e", nullptr));

  int emitted = 0;
  emcomm::debugIfEnabled(false, [&]() { ++emitted; });
  assert(emitted == 0);
  emcomm::debugIfEnabled(true, [&]() { ++emitted; });
  assert(emitted == 1);
}
