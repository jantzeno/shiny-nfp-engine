#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "packing/sparrow/optimize/separator.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct SeparatorWorkerResult {
  std::size_t worker_index{0};
  std::uint64_t worker_seed{0};
  SeparatorResult result{};
  runtime::TraceCapture trace{};
};

struct SeparatorWorkerRun {
  SeparatorWorkerResult best{};
  std::vector<SeparatorWorkerResult> workers{};
};

[[nodiscard]] auto run_separator_workers(const adapters::PortPolygon &container,
                                         std::span<const SeparatorItem> items,
                                         const SeparatorConfig &config,
                                         std::uint64_t worker_seed_base,
                                         std::size_t worker_count)
    -> SeparatorWorkerRun;

} // namespace shiny::nesting::pack::sparrow::optimize