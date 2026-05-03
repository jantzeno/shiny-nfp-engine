#pragma once

#include "internal/request_normalization.hpp"
#include "packing/sparrow/config.hpp"
#include "packing/sparrow/instance.hpp"
#include "util/status.hpp"

namespace shiny::nesting::pack::sparrow::adapters {

struct PortRequestAdapterResult {
  PortInstance instance{};
  SeedFlowPlan seed_flow{};
};

[[nodiscard]] auto to_port_instance(const NormalizedRequest &request)
    -> PortInstance;

[[nodiscard]] auto adapt_request(const NormalizedRequest &request,
                                 const SolveControl &control,
                                 SolveProfile profile = SolveProfile::balanced)
    -> PortRequestAdapterResult;

[[nodiscard]] auto adapt_request(const ProfileRequest &request,
                                 const ProfileSolveControl &control)
    -> std::expected<PortRequestAdapterResult, util::Status>;

} // namespace shiny::nesting::pack::sparrow::adapters