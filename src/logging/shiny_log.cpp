#include "logging/shiny_log.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <optional>
#include <string>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace shiny::log {
namespace {

constexpr const char *kLoggerName = "shiny";

std::shared_ptr<spdlog::logger> g_logger;

[[nodiscard]] auto ParseLevelFromEnvironment()
    -> std::optional<spdlog::level::level_enum> {
  const char *raw_level = std::getenv("SHINY_NESTING_LOG_LEVEL");
  if (raw_level == nullptr || *raw_level == '\0') {
    return std::nullopt;
  }

  std::string level(raw_level);
  std::transform(level.begin(), level.end(), level.begin(),
                 [](const unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });

  if (level == "trace") {
    return spdlog::level::trace;
  }
  if (level == "debug") {
    return spdlog::level::debug;
  }
  if (level == "info") {
    return spdlog::level::info;
  }
  if (level == "warn" || level == "warning") {
    return spdlog::level::warn;
  }
  if (level == "error") {
    return spdlog::level::err;
  }
  if (level == "critical") {
    return spdlog::level::critical;
  }
  if (level == "off") {
    return spdlog::level::off;
  }
  return std::nullopt;
}

} // namespace

void InitializeLogger() {
  if (g_logger != nullptr) {
    return;
  }

  g_logger = spdlog::get(kLoggerName);
  if (g_logger == nullptr) {
    g_logger = spdlog::stdout_color_mt(kLoggerName);
  }

  g_logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [t%t] [" +
                        std::string(kLoggerName) + "] %v");

  const std::optional<spdlog::level::level_enum> env_level =
      ParseLevelFromEnvironment();
  if (env_level.has_value()) {
    g_logger->set_level(*env_level);
  } else {
#ifdef NDEBUG
    g_logger->set_level(spdlog::level::info);
#else
    g_logger->set_level(spdlog::level::trace);
#endif
  }
  g_logger->flush_on(spdlog::level::debug);
  spdlog::set_default_logger(g_logger);
}

void ShutdownLogger() {
  if (g_logger == nullptr) {
    spdlog::drop(kLoggerName);
    return;
  }

  g_logger.reset();
  spdlog::drop(kLoggerName);
}

std::shared_ptr<spdlog::logger> GetLogger() {
  if (g_logger == nullptr) {
    InitializeLogger();
  }
  return g_logger;
}

} // namespace shiny::log
