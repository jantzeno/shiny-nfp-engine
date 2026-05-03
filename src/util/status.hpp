#pragma once

#include <expected>
#include <string_view>
#include <utility>

namespace shiny::nesting::util {

/**
 * @brief Minimal status code used by IO and utility helpers.
 */
enum class Status {
  ok = 0,
  invalid_input = 1,
  computation_failed = 2,
  cache_miss = 3,
  not_implemented = 4,
};

[[nodiscard]] constexpr auto status_name(const Status status)
    -> std::string_view {
  switch (status) {
  case Status::ok:
    return "ok";
  case Status::invalid_input:
    return "invalid_input";
  case Status::computation_failed:
    return "computation_failed";
  case Status::cache_miss:
    return "cache_miss";
  case Status::not_implemented:
    return "not_implemented";
  }
  return "unknown";
}

} // namespace shiny::nesting::util
