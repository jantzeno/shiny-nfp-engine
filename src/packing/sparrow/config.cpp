#include "packing/sparrow/config.hpp"

#include <array>
#include <chrono>

namespace shiny::nesting::pack::sparrow {

namespace {

constexpr std::uint64_t kConstructiveSalt = 0x9e3779b97f4a7c15ULL;
constexpr std::uint64_t kWorkerSalt = 0xd1b54a32d192ed03ULL;

constexpr std::array kParityDomains{
    ParityTestDomain::quantify,
    ParityTestDomain::tracker,
    ParityTestDomain::sampler,
    ParityTestDomain::separator,
    ParityTestDomain::constructive_seed,
    ParityTestDomain::exploration,
    ParityTestDomain::compression,
    ParityTestDomain::profile_balanced,
    ParityTestDomain::profile_maximum_search,
};

constexpr std::array kRuntimePhases{
    SparrowPhase::constructive_seed,
    SparrowPhase::separator,
    SparrowPhase::exploration,
    SparrowPhase::disruption,
    SparrowPhase::compression,
    SparrowPhase::profile_balanced,
    SparrowPhase::profile_maximum_search,
};

constexpr std::array kBudgetTerms{
    SparrowBudgetTerm::time_limit_milliseconds,
    SparrowBudgetTerm::operation_limit,
    SparrowBudgetTerm::separator_iterations,
    SparrowBudgetTerm::strike_limit,
    SparrowBudgetTerm::global_samples,
    SparrowBudgetTerm::focused_samples,
    SparrowBudgetTerm::coordinate_descent_iterations,
    SparrowBudgetTerm::exploration_iterations,
    SparrowBudgetTerm::compression_iterations,
};

constexpr std::array kPhaseBudgetBindings{
    PhaseBudgetBinding{.phase = SparrowPhase::constructive_seed,
                       .budget = SparrowBudgetTerm::time_limit_milliseconds},
    PhaseBudgetBinding{.phase = SparrowPhase::separator,
                       .budget = SparrowBudgetTerm::separator_iterations},
    PhaseBudgetBinding{.phase = SparrowPhase::separator,
                       .budget = SparrowBudgetTerm::strike_limit},
    PhaseBudgetBinding{.phase = SparrowPhase::separator,
                       .budget = SparrowBudgetTerm::global_samples},
    PhaseBudgetBinding{.phase = SparrowPhase::separator,
                       .budget = SparrowBudgetTerm::focused_samples},
    PhaseBudgetBinding{.phase = SparrowPhase::separator,
                       .budget =
                           SparrowBudgetTerm::coordinate_descent_iterations},
    PhaseBudgetBinding{.phase = SparrowPhase::exploration,
                       .budget = SparrowBudgetTerm::exploration_iterations},
    PhaseBudgetBinding{.phase = SparrowPhase::disruption,
                       .budget = SparrowBudgetTerm::operation_limit},
    PhaseBudgetBinding{.phase = SparrowPhase::compression,
                       .budget = SparrowBudgetTerm::compression_iterations},
    PhaseBudgetBinding{.phase = SparrowPhase::profile_balanced,
                       .budget = SparrowBudgetTerm::time_limit_milliseconds},
    PhaseBudgetBinding{.phase = SparrowPhase::profile_balanced,
                       .budget = SparrowBudgetTerm::operation_limit},
    PhaseBudgetBinding{.phase = SparrowPhase::profile_maximum_search,
                       .budget = SparrowBudgetTerm::time_limit_milliseconds},
    PhaseBudgetBinding{.phase = SparrowPhase::profile_maximum_search,
                       .budget = SparrowBudgetTerm::operation_limit},
};

[[nodiscard]] auto mix_seed(std::uint64_t value, const std::uint64_t salt)
    -> std::uint64_t {
  value += salt;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

[[nodiscard]] auto profile_seed_anchor(const SolveProfile profile)
    -> std::uint64_t {
  switch (profile) {
  case SolveProfile::quick:
    return 0x515549434bULL;
  case SolveProfile::balanced:
    return 0x42414c414e434544ULL;
  case SolveProfile::maximum_search:
    return 0x4d41585345415243ULL;
  }
  return 0x53484152524f57ULL;
}

constexpr std::array<PortLedgerEntry, 9> kPortLedger{{
    {
        .rust_source_path = "sparrow/quantify/overlap_proxy.rs",
        .cpp_destination_path =
            "src/packing/sparrow/quantify/overlap_proxy.cpp",
        .catch2_target = "tests/unit/sparrow/sparrow_quantify.cpp",
        .domain = ParityTestDomain::quantify,
        .adaptation = ReferenceAdaptation::direct_translation,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation =
            "Milestone 5 starts by fixing the quantifier destination before "
            "the overlap proxy translation lands.",
    },
    {
        .rust_source_path = "sparrow/quantify/collision_tracker.rs",
        .cpp_destination_path =
            "src/packing/sparrow/quantify/collision_tracker.cpp",
        .catch2_target = "tests/unit/sparrow/sparrow_tracker.cpp",
        .domain = ParityTestDomain::tracker,
        .adaptation = ReferenceAdaptation::fixture_adaptation,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation =
            "Milestone 4 captures tracker-facing trace events before the "
            "CollisionTracker translation lands.",
    },
    {
        .rust_source_path = "sparrow/sample/uniform_sampler.rs",
        .cpp_destination_path =
            "src/packing/sparrow/sample/uniform_sampler.cpp",
        .catch2_target = "tests/unit/sparrow/sparrow_sampling.cpp",
        .domain = ParityTestDomain::sampler,
        .adaptation = ReferenceAdaptation::direct_translation,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation = "Milestone 4 standardizes deterministic seed flow before "
                     "sampler translation.",
    },
    {
        .rust_source_path = "sparrow/optimize/separator.rs",
        .cpp_destination_path = "src/packing/sparrow/optimize/separator.cpp",
        .catch2_target = "tests/unit/sparrow/sparrow_separator.cpp",
        .domain = ParityTestDomain::separator,
        .adaptation = ReferenceAdaptation::behavior_invariant,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation =
            "Milestone 4 captures separator strike and compaction traces "
            "before separator parity lands.",
    },
    {
        .rust_source_path = "sparrow/constructive/seed_solution.rs",
        .cpp_destination_path =
            "src/packing/sparrow/eval/constructive_seed_evaluator.cpp",
        .catch2_target = "tests/unit/sparrow/sparrow_constructive_seed.cpp",
        .domain = ParityTestDomain::constructive_seed,
        .adaptation = ReferenceAdaptation::fixture_adaptation,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation =
            "Milestone 4 seeds Sparrow from the fill-first constructive layout "
            "instead of Sparrow's original strip seed builder.",
    },
    {
        .rust_source_path = "sparrow/optimize/exploration_phase.rs",
        .cpp_destination_path =
            "src/packing/sparrow/optimize/exploration_phase.cpp",
        .catch2_target = "tests/integration/profiles/balanced.cpp",
        .domain = ParityTestDomain::exploration,
        .adaptation = ReferenceAdaptation::behavior_invariant,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation =
            "Milestone 4 keeps exploration parity at the behavior level until "
            "the exploration-phase source group lands.",
    },
    {
        .rust_source_path = "sparrow/optimize/compression_phase.rs",
        .cpp_destination_path =
            "src/packing/sparrow/optimize/compression_phase.cpp",
        .catch2_target = "tests/integration/profiles/maximum_search.cpp",
        .domain = ParityTestDomain::compression,
        .adaptation = ReferenceAdaptation::behavior_invariant,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation = "Milestone 4 records compression-attempt contracts and "
                     "parity fixtures before the compression port begins.",
    },
    {
        .rust_source_path = "sparrow/api/profile_balanced.rs",
        .cpp_destination_path = "src/solve.cpp",
        .catch2_target = "tests/integration/profiles/balanced.cpp",
        .domain = ParityTestDomain::profile_balanced,
        .adaptation = ReferenceAdaptation::behavior_invariant,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation = "Balanced remains profile-driven through SolveProfile "
                     "rather than a direct Rust API mirror.",
    },
    {
        .rust_source_path = "sparrow/api/profile_maximum_search.rs",
        .cpp_destination_path = "src/solve.cpp",
        .catch2_target = "tests/integration/profiles/maximum_search.cpp",
        .domain = ParityTestDomain::profile_maximum_search,
        .adaptation = ReferenceAdaptation::behavior_invariant,
        .license = "MIT",
        .attribution = "sparrow",
        .deviation = "Maximum Search remains profile-driven through "
                     "SolveProfile rather than a direct Rust API mirror.",
    },
}};

[[nodiscard]] auto seed_flow_plan(const std::uint64_t public_seed,
                                  const SeedProgressionMode seed_mode,
                                  const SolveProfile profile) -> SeedFlowPlan {
  std::uint64_t root_seed =
      public_seed == 0 ? profile_seed_anchor(profile) : public_seed;
  if (seed_mode == SeedProgressionMode::random) {
    root_seed ^= static_cast<std::uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
  }
  return {
      .profile = profile,
      .seed_mode = seed_mode,
      .rng_kind = DeterministicRngKind::splitmix64,
      .public_seed = public_seed,
      .constructive_seed =
          mix_seed(root_seed, profile_seed_anchor(profile) ^ kConstructiveSalt),
      .worker_seed_base =
          mix_seed(root_seed, profile_seed_anchor(profile) ^ kWorkerSalt),
  };
}

} // namespace

auto parity_test_domains() -> std::span<const ParityTestDomain> {
  return kParityDomains;
}

auto parity_domain_name(const ParityTestDomain domain) -> std::string_view {
  switch (domain) {
  case ParityTestDomain::quantify:
    return "quantify";
  case ParityTestDomain::tracker:
    return "tracker";
  case ParityTestDomain::sampler:
    return "sampler";
  case ParityTestDomain::separator:
    return "separator";
  case ParityTestDomain::constructive_seed:
    return "constructive_seed";
  case ParityTestDomain::exploration:
    return "exploration";
  case ParityTestDomain::compression:
    return "compression";
  case ParityTestDomain::profile_balanced:
    return "profile_balanced";
  case ParityTestDomain::profile_maximum_search:
    return "profile_maximum_search";
  }
  return "unknown";
}

auto sparrow_runtime_phases() -> std::span<const SparrowPhase> {
  return kRuntimePhases;
}

auto runtime_phase_name(const SparrowPhase phase) -> std::string_view {
  switch (phase) {
  case SparrowPhase::constructive_seed:
    return "constructive_seed";
  case SparrowPhase::separator:
    return "separator";
  case SparrowPhase::exploration:
    return "exploration";
  case SparrowPhase::disruption:
    return "disruption";
  case SparrowPhase::compression:
    return "compression";
  case SparrowPhase::profile_balanced:
    return "profile_balanced";
  case SparrowPhase::profile_maximum_search:
    return "profile_maximum_search";
  }
  return "unknown";
}

auto sparrow_budget_terms() -> std::span<const SparrowBudgetTerm> {
  return kBudgetTerms;
}

auto budget_term_name(const SparrowBudgetTerm budget) -> std::string_view {
  switch (budget) {
  case SparrowBudgetTerm::time_limit_milliseconds:
    return "time_limit_milliseconds";
  case SparrowBudgetTerm::operation_limit:
    return "operation_limit";
  case SparrowBudgetTerm::separator_iterations:
    return "separator_iterations";
  case SparrowBudgetTerm::strike_limit:
    return "strike_limit";
  case SparrowBudgetTerm::global_samples:
    return "global_samples";
  case SparrowBudgetTerm::focused_samples:
    return "focused_samples";
  case SparrowBudgetTerm::coordinate_descent_iterations:
    return "coordinate_descent_iterations";
  case SparrowBudgetTerm::exploration_iterations:
    return "exploration_iterations";
  case SparrowBudgetTerm::compression_iterations:
    return "compression_iterations";
  }
  return "unknown";
}

auto phase_budget_vocabulary() -> std::span<const PhaseBudgetBinding> {
  return kPhaseBudgetBindings;
}

auto port_ledger() -> std::span<const PortLedgerEntry> { return kPortLedger; }

auto build_seed_flow_plan(const SolveControl &control,
                          const SolveProfile profile) -> SeedFlowPlan {
  return seed_flow_plan(control.random_seed, control.seed_mode, profile);
}

auto build_seed_flow_plan(const ProfileSolveControl &control,
                          const SolveProfile profile) -> SeedFlowPlan {
  return seed_flow_plan(control.random_seed, control.seed_mode, profile);
}

} // namespace shiny::nesting::pack::sparrow