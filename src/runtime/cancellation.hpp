#pragma once

#include <atomic>
#include <memory>

#include "packing/decoder.hpp"

namespace shiny::nesting::runtime {

class CancellationToken {
public:
  CancellationToken() = default;
  explicit CancellationToken(std::shared_ptr<std::atomic<bool>> flag)
      : flag_(std::move(flag)) {}

  [[nodiscard]] auto stop_requested() const -> bool {
    return flag_ != nullptr && flag_->load();
  }

private:
  std::shared_ptr<std::atomic<bool>> flag_{};
};

class CancellationSource {
public:
  CancellationSource() : flag_(std::make_shared<std::atomic<bool>>(false)) {}

  [[nodiscard]] auto token() const -> CancellationToken {
    return CancellationToken(flag_);
  }

  void request_stop() const {
    if (flag_ != nullptr) {
      flag_->store(true);
    }
  }

private:
  std::shared_ptr<std::atomic<bool>> flag_{};
};

[[nodiscard]] inline auto make_interruption_probe(
    const CancellationToken &token) -> pack::InterruptionProbe {
  return [token]() { return token.stop_requested(); };
}

} // namespace shiny::nesting::runtime
