#pragma once

namespace emcomm {

template <typename Ota, typename Start, typename End, typename Progress,
          typename Error>
void setArduinoOtaCallbacks(Ota &ota, Start start, End end, Progress progress,
                           Error error) {
  ota.onStart(start);
  ota.onEnd(end);
  ota.onProgress(progress);
  ota.onError(error);
}

template <typename Ota>
void startArduinoOta(Ota &ota, const char *hostname, const char *password) {
  ota.setHostname(hostname);
  if (password != nullptr && password[0] != '\0')
    ota.setPassword(password);
  ota.begin();
}

} // namespace emcomm
