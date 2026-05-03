#pragma once

#include <stop_token>

#include "runtime/interruption.hpp"

namespace shiny::nesting::runtime {

/**
 * @brief Lightweight cancellation observer backed by std::stop_token.
 *
 * Wraps std::stop_token to preserve the existing stop_requested() polling
 * interface while removing the shared_ptr<atomic<bool>> implementation. A
 * default-constructed token is always non-stopping, matching the semantics of
 * a default-constructed std::stop_token.
 */
class CancellationToken {
public:
  CancellationToken() = default;
  explicit CancellationToken(std::stop_token token)
      : token_(std::move(token)) {}

  [[nodiscard]] auto stop_requested() const -> bool {
    return token_.stop_requested();
  }

private:
  std::stop_token token_{};
};

/**
 * @brief Cancellation owner backed by std::stop_source.
 *
 * Wraps std::stop_source to preserve the existing token() / request_stop()
 * interface while removing the shared_ptr<atomic<bool>> implementation. The
 * stop state is shared between all CancellationTokens produced by token().
 */
class CancellationSource {
public:
  CancellationSource() = default;

  [[nodiscard]] auto token() const -> CancellationToken {
    return CancellationToken(source_.get_token());
  }

  void request_stop() { source_.request_stop(); }

private:
  std::stop_source source_{};
};

[[nodiscard]] inline auto
make_interruption_probe(const CancellationToken &token)
    -> InterruptionProbe {
  return [token]() { return token.stop_requested(); };
}

} // namespace shiny::nesting::runtime
