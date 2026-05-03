#pragma once

#include <cstdint>
#include <span>
#include <string_view>

#include "solve.hpp"

namespace shiny::nesting::pack::sparrow {

enum class ParityTestDomain : std::uint8_t {
  quantify = 0,
  tracker = 1,
  sampler = 2,
  separator = 3,
  constructive_seed = 4,
  exploration = 5,
  compression = 6,
  profile_balanced = 7,
  profile_maximum_search = 8,
};

enum class ReferenceAdaptation : std::uint8_t {
  direct_translation = 0,
  fixture_adaptation = 1,
  behavior_invariant = 2,
};

enum class DeterministicRngKind : std::uint8_t {
  splitmix64 = 0,
};

enum class SparrowPhase : std::uint8_t {
  constructive_seed = 0,
  separator = 1,
  exploration = 2,
  disruption = 3,
  compression = 4,
  profile_balanced = 5,
  profile_maximum_search = 6,
};

enum class SparrowBudgetTerm : std::uint8_t {
  time_limit_milliseconds = 0,
  operation_limit = 1,
  separator_iterations = 2,
  strike_limit = 3,
  global_samples = 4,
  focused_samples = 5,
  coordinate_descent_iterations = 6,
  exploration_iterations = 7,
  compression_iterations = 8,
};

struct PhaseBudgetBinding {
  SparrowPhase phase{SparrowPhase::constructive_seed};
  SparrowBudgetTerm budget{SparrowBudgetTerm::time_limit_milliseconds};
};

struct PortLedgerEntry {
  std::string_view rust_source_path{};
  std::string_view cpp_destination_path{};
  std::string_view catch2_target{};
  ParityTestDomain domain{ParityTestDomain::quantify};
  ReferenceAdaptation adaptation{ReferenceAdaptation::direct_translation};
  std::string_view license{"MIT"};
  std::string_view attribution{"sparrow"};
  std::string_view deviation{};
};

struct SeedFlowPlan {
  SolveProfile profile{SolveProfile::balanced};
  SeedProgressionMode seed_mode{SeedProgressionMode::increment};
  DeterministicRngKind rng_kind{DeterministicRngKind::splitmix64};
  std::uint64_t public_seed{0};
  std::uint64_t constructive_seed{0};
  std::uint64_t worker_seed_base{0};
};

[[nodiscard]] auto parity_test_domains() -> std::span<const ParityTestDomain>;

[[nodiscard]] auto parity_domain_name(ParityTestDomain domain)
    -> std::string_view;

[[nodiscard]] auto sparrow_runtime_phases() -> std::span<const SparrowPhase>;

[[nodiscard]] auto runtime_phase_name(SparrowPhase phase) -> std::string_view;

[[nodiscard]] auto sparrow_budget_terms() -> std::span<const SparrowBudgetTerm>;

[[nodiscard]] auto budget_term_name(SparrowBudgetTerm budget)
    -> std::string_view;

[[nodiscard]] auto phase_budget_vocabulary()
    -> std::span<const PhaseBudgetBinding>;

[[nodiscard]] auto port_ledger() -> std::span<const PortLedgerEntry>;

[[nodiscard]] auto
build_seed_flow_plan(const SolveControl &control,
                     SolveProfile profile = SolveProfile::balanced)
    -> SeedFlowPlan;

[[nodiscard]] auto build_seed_flow_plan(const ProfileSolveControl &control,
                                        SolveProfile profile) -> SeedFlowPlan;

} // namespace shiny::nesting::pack::sparrow