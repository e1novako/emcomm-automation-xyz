#pragma once

namespace emcomm {

template <typename Emit>
inline void debugIfEnabled(bool enabled, Emit emit) {
  if (enabled)
    emit();
}

} // namespace emcomm
