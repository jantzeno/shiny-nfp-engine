#pragma once

#include <cstdint>
#include <optional>

namespace shiny::nesting::pack {

enum class BinLifecycle : std::uint8_t {
  user_created = 0,
  engine_overflow = 1,
};

struct BinIdentity {
  BinLifecycle lifecycle{BinLifecycle::user_created};
  std::uint32_t source_request_bin_id{0};
  std::optional<std::uint32_t> template_bin_id{};
};

} // namespace shiny::nesting::pack