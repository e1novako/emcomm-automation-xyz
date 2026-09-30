#include "../libraries/EmcommCommon/src/EmcommCommon/Diagnostics.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/ArduinoOta.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/MacAddress.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/OtaUpload.h"
#include "../libraries/EmcommCommon/src/EmcommCommon/Web.h"

#include <assert.h>
#include <functional>
#include <stdint.h>
#include <string>

struct FakeOtaBackend {
  bool beginResult = true;
  bool finishResult = true;
  size_t writeResult = 3;
  int beginCalls = 0, writeCalls = 0, finishCalls = 0, abortCalls = 0;

  bool begin() {
    ++beginCalls;
    return beginResult;
  }
  size_t write(const uint8_t *, size_t length) {
    ++writeCalls;
    return writeResult == 3 ? length : writeResult;
  }
  bool finish() {
    ++finishCalls;
    return finishResult;
  }
  void abort() { ++abortCalls; }
};

struct FakeArduinoOta {
  std::string hostname, password;
  int beginCalls = 0;
  void setHostname(const char *value) { hostname = value; }
  void setPassword(const char *value) { password = value; }
  void begin() { ++beginCalls; }

  template <typename Callback> void onStart(Callback callback) {
    start = callback;
  }
  template <typename Callback> void onEnd(Callback callback) { end = callback; }
  template <typename Callback>
  void onProgress(Callback callback) {
    progress = callback;
  }
  template <typename Callback> void onError(Callback callback) {
    error = callback;
  }

  std::function<void()> start, end;
  std::function<void(unsigned int, unsigned int)> progress;
  std::function<void(int)> error;
};

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

  assert(emcomm::htmlEscape(std::string("&<>\"'")) ==
         "&amp;&lt;&gt;&quot;&#39;");
  assert(emcomm::htmlEscape(std::string("plain text")) == "plain text");

  const uint8_t payload[] = {1, 2, 3};
  FakeOtaBackend backend;
  emcomm::OtaUpload<FakeOtaBackend> upload;
  assert(upload.begin(backend) == emcomm::OtaUploadState::Receiving);
  assert(upload.write(backend, payload, sizeof(payload)) ==
         emcomm::OtaUploadState::Receiving);
  assert(upload.finish(backend) == emcomm::OtaUploadState::Complete);
  assert(upload.succeeded());
  assert(backend.beginCalls == 1 && backend.writeCalls == 1 &&
         backend.finishCalls == 1 && backend.abortCalls == 0);

  backend = FakeOtaBackend();
  backend.beginResult = false;
  assert(upload.begin(backend) == emcomm::OtaUploadState::BeginFailed);
  assert(upload.write(backend, payload, sizeof(payload)) ==
         emcomm::OtaUploadState::BeginFailed);
  assert(backend.writeCalls == 0 && backend.finishCalls == 0);

  backend = FakeOtaBackend();
  backend.writeResult = 1;
  assert(upload.begin(backend) == emcomm::OtaUploadState::Receiving);
  assert(upload.write(backend, payload, sizeof(payload)) ==
         emcomm::OtaUploadState::WriteFailed);
  assert(backend.abortCalls == 1);

  backend = FakeOtaBackend();
  backend.finishResult = false;
  assert(upload.begin(backend) == emcomm::OtaUploadState::Receiving);
  assert(upload.finish(backend) == emcomm::OtaUploadState::FinishFailed);

  backend = FakeOtaBackend();
  assert(upload.begin(backend) == emcomm::OtaUploadState::Receiving);
  assert(upload.abort(backend) == emcomm::OtaUploadState::Aborted);
  assert(backend.abortCalls == 1);

  FakeArduinoOta arduinoOta;
  int callbackCount = 0;
  emcomm::setArduinoOtaCallbacks(
      arduinoOta, [&]() { ++callbackCount; }, [&]() { ++callbackCount; },
      [&](unsigned int, unsigned int) { ++callbackCount; },
      [&](int) { ++callbackCount; });
  emcomm::startArduinoOta(arduinoOta, "device", "password");
  assert(arduinoOta.hostname == "device");
  assert(arduinoOta.password == "password");
  assert(arduinoOta.beginCalls == 1);
  arduinoOta.start();
  arduinoOta.end();
  arduinoOta.progress(1, 2);
  arduinoOta.error(1);
  assert(callbackCount == 4);

  emcomm::startArduinoOta(arduinoOta, "device-2", "");
  assert(arduinoOta.hostname == "device-2");
  assert(arduinoOta.password == "password");
  assert(arduinoOta.beginCalls == 2);
}
