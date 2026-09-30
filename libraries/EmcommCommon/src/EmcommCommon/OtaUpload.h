#pragma once

#include <stddef.h>
#include <stdint.h>

namespace emcomm {

enum class OtaUploadState : uint8_t {
  Idle,
  Receiving,
  Complete,
  BeginFailed,
  WriteFailed,
  FinishFailed,
  Aborted
};

template <typename Backend> class OtaUpload {
public:
  OtaUploadState begin(Backend &backend) {
    state_ = backend.begin() ? OtaUploadState::Receiving
                             : OtaUploadState::BeginFailed;
    return state_;
  }

  OtaUploadState write(Backend &backend, const uint8_t *data, size_t length) {
    if (state_ != OtaUploadState::Receiving)
      return state_;
    if ((data == nullptr && length != 0) ||
        backend.write(data, length) != length) {
      backend.abort();
      state_ = OtaUploadState::WriteFailed;
    }
    return state_;
  }

  OtaUploadState finish(Backend &backend) {
    if (state_ != OtaUploadState::Receiving)
      return state_;
    state_ = backend.finish() ? OtaUploadState::Complete
                              : OtaUploadState::FinishFailed;
    return state_;
  }

  OtaUploadState abort(Backend &backend) {
    if (state_ == OtaUploadState::Receiving)
      backend.abort();
    state_ = OtaUploadState::Aborted;
    return state_;
  }

  OtaUploadState state() const { return state_; }

  bool succeeded() const { return state_ == OtaUploadState::Complete; }

private:
  OtaUploadState state_ = OtaUploadState::Idle;
};

} // namespace emcomm
