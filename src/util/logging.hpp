#pragma once

#include <utility>

#if defined(SHINY_NFP_ENABLE_LOGGING)
#include <iostream>
#endif

namespace shiny::nfp::util {

/**
 * @brief Logging severity used by the lightweight repo logging macro.
 */
enum class LogLevel {
  trace = 0,
  debug = 1,
  info = 2,
  warn = 3,
  error = 4,
};

namespace detail {

/**
 * @brief Emits one log line when logging is enabled at compile time.
 *
 * @param level Message severity.
 * @param args Streamed message fragments.
 */
template <class... Args>
inline auto log_message(LogLevel level, Args &&...args) -> void {
#if defined(SHINY_NFP_ENABLE_LOGGING)
  std::clog << '[' << static_cast<int>(level) << "] ";
  (std::clog << ... << std::forward<Args>(args));
  std::clog << '\n';
#else
  (void)level;
  (void)sizeof...(args);
#endif
}

} // namespace detail

} // namespace shiny::nfp::util

/**
 * @brief Convenience macro for compile-time gated repo logging.
 */
#define SHINY_NFP_LOG(level, ...)                                              \
  do {                                                                         \
    ::shiny::nfp::util::detail::log_message(                                   \
        ::shiny::nfp::util::LogLevel::level, __VA_ARGS__);                     \
  } while (false)
