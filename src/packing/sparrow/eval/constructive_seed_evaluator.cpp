#include "packing/sparrow/eval/constructive_seed_evaluator.hpp"

#include <bit>

namespace shiny::nesting::pack::sparrow::eval {

namespace {

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

auto mix_hash(std::uint64_t hash, const std::uint64_t value) -> std::uint64_t {
  return (hash ^ value) * kFnvPrime;
}

auto mix_double(std::uint64_t hash, const double value) -> std::uint64_t {
  return mix_hash(hash, std::bit_cast<std::uint64_t>(value));
}

auto compute_utilization_percent(const Layout &layout) -> double {
  double total_occupied_area = 0.0;
  double total_container_area = 0.0;
  for (const auto &bin : layout.bins) {
    total_occupied_area += bin.utilization.occupied_area;
    total_container_area += bin.utilization.container_area;
  }
  if (total_container_area <= 0.0) {
    return 0.0;
  }
  return (total_occupied_area / total_container_area) * 100.0;
}

auto terminal_bin_id(const SeedSolution &seed) -> std::optional<std::uint32_t> {
  if (!seed.placements.empty()) {
    return seed.placements.back().bin_id;
  }
  if (!seed.constructive.frontier_changes.empty()) {
    return seed.constructive.frontier_changes.back().next_bin_id;
  }
  return std::nullopt;
}

auto build_warm_start_signature(const SeedSolution &seed) -> std::uint64_t {
  std::uint64_t hash = kFnvOffsetBasis;
  hash = mix_hash(hash, seed.placements.size());
  hash = mix_hash(hash, seed.unplaced_piece_ids.size());

  for (const auto &placement : seed.placements) {
    hash = mix_hash(hash, placement.piece_id);
    hash = mix_hash(hash, placement.bin_id);
    hash = mix_double(hash, placement.translation.x());
    hash = mix_double(hash, placement.translation.y());
    hash = mix_double(hash, placement.resolved_rotation.degrees);
    hash = mix_hash(hash, placement.mirrored ? 1U : 0U);
    hash = mix_hash(hash, static_cast<std::uint64_t>(placement.phase));
  }

  for (const auto piece_id : seed.unplaced_piece_ids) {
    hash = mix_hash(hash, piece_id);
  }

  for (const auto &change : seed.constructive.frontier_changes) {
    hash = mix_hash(hash, change.previous_bin_id.value_or(0U));
    hash = mix_hash(hash, change.next_bin_id);
    hash = mix_hash(hash, change.piece_id);
    hash = mix_hash(hash, static_cast<std::uint64_t>(change.reason));
  }

  for (const auto &overflow : seed.constructive.overflow_events) {
    hash = mix_hash(hash, overflow.template_bin_id);
    hash = mix_hash(hash, overflow.overflow_bin_id);
    hash = mix_hash(hash, overflow.source_request_bin_id);
  }

  for (const auto &exhaustion : seed.constructive.exhaustion_events) {
    hash = mix_hash(hash, exhaustion.piece_id);
    hash = mix_hash(hash, exhaustion.frontier_bin_id);
    hash = mix_hash(hash, static_cast<std::uint64_t>(exhaustion.decision));
  }

  return hash;
}

} // namespace

auto evaluate_constructive_seed(const SeedSolution &seed)
    -> ConstructiveSeedEvaluation {
  const auto utilization_percent = compute_utilization_percent(seed.layout);
  const auto placement_count = seed.placements.size();
  const auto unplaced_count = seed.unplaced_piece_ids.size();
  const auto frontier_change_count = seed.constructive.frontier_changes.size();
  const auto overflow_event_count = seed.constructive.overflow_events.size();
  const auto exhaustion_event_count =
      seed.constructive.exhaustion_events.size();

  return {
      .score = static_cast<double>(placement_count) * 1'000'000.0 -
               static_cast<double>(unplaced_count) * 10'000.0 -
               static_cast<double>(overflow_event_count) * 100.0 -
               static_cast<double>(exhaustion_event_count) +
               utilization_percent,
      .utilization_percent = utilization_percent,
      .complete_warm_start = placement_count > 0U && unplaced_count == 0U,
      .warm_start_signature = build_warm_start_signature(seed),
      .replay =
          {
              .placement_count = placement_count,
              .unplaced_count = unplaced_count,
              .frontier_change_count = frontier_change_count,
              .overflow_event_count = overflow_event_count,
              .exhaustion_event_count = exhaustion_event_count,
              .terminal_bin_id = terminal_bin_id(seed),
          },
  };
}

} // namespace shiny::nesting::pack::sparrow::eval