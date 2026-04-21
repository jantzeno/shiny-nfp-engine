#pragma once

#include <cstddef>
#include <cstdint>

#include "observer.hpp"
#include "request.hpp"
#include "result.hpp"
#include "runtime/cancellation.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

namespace pack {
struct PackerWorkspace;
}

enum class SeedProgressionMode : std::uint8_t {
  increment = 0,
  decrement = 1,
  random = 2,
};

struct SolveControl {
  ProgressObserver on_progress{};
  runtime::CancellationToken cancellation{};
  std::size_t iteration_limit{0};
  std::uint64_t time_limit_milliseconds{0};
  std::uint64_t random_seed{0};
  SeedProgressionMode seed_mode{SeedProgressionMode::increment};
  pack::PackerWorkspace *workspace{nullptr};
};

[[nodiscard]] auto solve(const NestingRequest &request,
                         const SolveControl &control = {})
    -> util::StatusOr<NestingResult>;

} // namespace shiny::nesting
