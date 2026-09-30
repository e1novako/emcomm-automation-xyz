# EmcommCommon

Header-only, platform-independent helpers shared by the firmware projects.

- `EmcommCommon/MacAddress.h` parses exactly six colon-separated hexadecimal
  octets. On failure it returns `false` without modifying the output buffer.
- `EmcommCommon/Diagnostics.h` invokes a supplied debug-output callback only
  while that project's existing GUI-controlled setting is enabled. Essential
  information and error logging remain the responsibility of each project.
- `EmcommCommon/OtaUpload.h` sequences backend begin/write/finish/abort calls
  and exposes the resulting upload state; projects supply platform-specific
  update backends.
- `EmcommCommon/ArduinoOta.h` registers project-supplied callbacks and starts
  ArduinoOTA with the configured hostname and optional password.
- `EmcommCommon/Web.h` provides HTML escaping for Arduino `String` and
  compatible string types.

The MAC parser is consumed by C4-NODEMCU and VIBRANT. The debug gate is
consumed by C4-NODEMCU, VIBRANT, and Vibrant-ESP32CAM. GUI controls and stored
configuration remain project-specific. The OTA upload state machine is used
by all three projects; ArduinoOTA setup is used by VIBRANT and
Vibrant-ESP32CAM. HTML escaping is used by C4-NODEMCU and VIBRANT. Web-server
routes, authentication, responses, page assets, and update target/partition
selection remain project-specific because the projects use different HTTP
servers and hardware policies.

Run the host regression tests from the repository root with:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  tests/emcomm_common_test.cpp -o /tmp/emcomm_common_test
/tmp/emcomm_common_test
```
