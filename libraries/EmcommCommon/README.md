# EmcommCommon

Header-only, platform-independent helpers shared by the firmware projects.

- `EmcommCommon/MacAddress.h` parses exactly six colon-separated hexadecimal
  octets. On failure it returns `false` without modifying the output buffer.
- `EmcommCommon/Diagnostics.h` invokes a supplied debug-output callback only
  while that project's existing GUI-controlled setting is enabled. Essential
  information and error logging remain the responsibility of each project.

The MAC parser is consumed by C4-NODEMCU and VIBRANT. The debug gate is
consumed by C4-NODEMCU, VIBRANT, and Vibrant-ESP32CAM. GUI controls and stored
configuration remain project-specific.

Run the host regression tests from the repository root with:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  tests/emcomm_common_test.cpp -o /tmp/emcomm_common_test
/tmp/emcomm_common_test
```
