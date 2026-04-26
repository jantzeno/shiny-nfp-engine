#include "search/detail/neighborhood_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry/polygon.hpp"
#include "packing/irregular/constructive_packer.hpp"
#include "runtime/hash.hpp"
#include "search/detail/layout_validation.hpp"
#include "search/disruption.hpp"

namespace shiny::nesting::search::detail {

auto piece_areas_for(const NormalizedRequest &request) -> std::vector<double> {
  std::unordered_map<std::uint32_t, double> areas_by_source_piece;
  areas_by_source_piece.reserve(request.request.pieces.size());
  for (const auto &piece : request.request.pieces) {
    areas_by_source_piece.emplace(piece.piece_id,
                                  geom::polygon_area(piece.polygon));
  }

  std::vector<double> areas;
  areas.reserve(request.expanded_pieces.size());
  for (const auto &expanded_piece : request.expanded_pieces) {
    const auto it = areas_by_source_piece.find(expanded_piece.source_piece_id);
    areas.push_back(it == areas_by_source_piece.end() ? 0.0 : it->second);
  }
  return areas;
}

namespace {

enum class DestroyStrategy : std::uint8_t {
  random = 0,
  area = 1,
  related = 2,
  cluster = 3,
};

struct RemovedPiece {
  std::size_t piece{0};
  std::optional<geom::RotationIndex> forced_rotation{};
};

[[nodiscard]] auto piece_rotation_counts_for(const NormalizedRequest &request)
    -> std::vector<std::size_t> {
  std::unordered_map<std::uint32_t, std::size_t> counts_by_source_piece;
  counts_by_source_piece.reserve(request.request.pieces.size());
  for (const auto &piece : request.request.pieces) {
    const auto &rotations = piece.allowed_rotations.has_value()
                                ? *piece.allowed_rotations
                                : request.request.execution.default_rotations;
    counts_by_source_piece.emplace(piece.piece_id,
                                   geom::rotation_count(rotations));
  }

  std::vector<std::size_t> counts;
  counts.reserve(request.expanded_pieces.size());
  for (const auto &expanded_piece : request.expanded_pieces) {
    const auto it = counts_by_source_piece.find(expanded_piece.source_piece_id);
    counts.push_back(it == counts_by_source_piece.end() ? 0U : it->second);
  }
  return counts;
}

[[nodiscard]] auto seed_from_order(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    const std::uint64_t base_seed, const std::uint64_t seed_bias)
    -> std::uint64_t {
  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  for (const auto index : order) {
    runtime::hash::fnv1a_mix_value(hash, index);
  }
  for (const auto &rotation : forced_rotations) {
    const std::uint64_t encoded =
        rotation.has_value()
            ? (static_cast<std::uint64_t>(rotation->value) + 1U)
            : 0ULL;
    runtime::hash::fnv1a_mix_value(hash, encoded);
  }
  return hash ^ base_seed ^ (seed_bias * runtime::hash::kGoldenRatio64);
}

[[nodiscard]] auto reordered_request_for(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    const NormalizedRequest &request) -> NormalizedRequest {
  NormalizedRequest reordered = request;
  reordered.expanded_pieces.clear();
  reordered.forced_rotations.clear();
  reordered.expanded_pieces.reserve(order.size());
  reordered.forced_rotations.reserve(order.size());
  for (const auto index : order) {
    reordered.expanded_pieces.push_back(request.expanded_pieces[index]);
    reordered.forced_rotations.push_back(index < forced_rotations.size()
                                             ? forced_rotations[index]
                                             : std::nullopt);
  }
  return reordered;
}

auto swap_positions(
    std::vector<std::size_t> &order,
    std::vector<std::optional<geom::RotationIndex>> &forced_rotations,
    const std::size_t lhs, const std::size_t rhs) -> bool {
  if (order.size() < 2U || lhs >= order.size() || rhs >= order.size() ||
      lhs == rhs) {
    return false;
  }
  std::swap(order[lhs], order[rhs]);
  if (forced_rotations.size() == order.size()) {
    std::swap(forced_rotations[lhs], forced_rotations[rhs]);
  }
  return true;
}

[[nodiscard]] auto clamp_destroy_count(const std::size_t piece_count,
                                       const std::size_t intensity)
    -> std::size_t {
  if (piece_count < 2U) {
    return 0U;
  }
  return std::clamp<std::size_t>(intensity, 1U,
                                 std::max<std::size_t>(1U, piece_count - 1U));
}

[[nodiscard]] auto area_for_piece(std::span<const double> piece_areas,
                                  const std::size_t piece_index) -> double {
  return piece_index < piece_areas.size() ? piece_areas[piece_index] : 0.0;
}

[[nodiscard]] auto select_removal_positions(std::span<const std::size_t> order,
                                            std::span<const double> piece_areas,
                                            runtime::DeterministicRng &rng,
                                            const std::size_t destroy_count,
                                            const DestroyStrategy strategy)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> removal_positions(order.size());
  std::iota(removal_positions.begin(), removal_positions.end(), 0U);

  switch (strategy) {
  case DestroyStrategy::area:
    std::stable_sort(removal_positions.begin(), removal_positions.end(),
                     [&](const std::size_t lhs, const std::size_t rhs) {
                       if (area_for_piece(piece_areas, order[lhs]) !=
                           area_for_piece(piece_areas, order[rhs])) {
                         return area_for_piece(piece_areas, order[lhs]) >
                                area_for_piece(piece_areas, order[rhs]);
                       }
                       return lhs < rhs;
                     });
    removal_positions.resize(destroy_count);
    break;
  case DestroyStrategy::related: {
    const auto seed_position = rng.uniform_index(order.size());
    const auto seed_area = area_for_piece(piece_areas, order[seed_position]);
    std::stable_sort(
        removal_positions.begin(), removal_positions.end(),
        [&](const std::size_t lhs, const std::size_t rhs) {
          const auto lhs_area = area_for_piece(piece_areas, order[lhs]);
          const auto rhs_area = area_for_piece(piece_areas, order[rhs]);
          const auto lhs_score =
              std::abs(std::log((lhs_area + 1e-9) / (seed_area + 1e-9))) +
              (0.15 * static_cast<double>(lhs > seed_position
                                              ? lhs - seed_position
                                              : seed_position - lhs));
          const auto rhs_score =
              std::abs(std::log((rhs_area + 1e-9) / (seed_area + 1e-9))) +
              (0.15 * static_cast<double>(rhs > seed_position
                                              ? rhs - seed_position
                                              : seed_position - rhs));
          if (lhs_score != rhs_score) {
            return lhs_score < rhs_score;
          }
          return lhs < rhs;
        });
    removal_positions.resize(destroy_count);
    break;
  }
  case DestroyStrategy::cluster: {
    const auto start = rng.uniform_index(order.size());
    removal_positions.clear();
    removal_positions.reserve(destroy_count);
    for (std::size_t offset = 0; offset < destroy_count; ++offset) {
      removal_positions.push_back((start + offset) % order.size());
    }
    break;
  }
  case DestroyStrategy::random:
    rng.shuffle(removal_positions);
    removal_positions.resize(destroy_count);
    break;
  }

  std::sort(removal_positions.begin(), removal_positions.end());
  return removal_positions;
}

[[nodiscard]] auto insertion_cost(std::span<const std::size_t> order,
                                  const std::size_t piece,
                                  std::span<const double> piece_areas,
                                  const std::size_t insert_pos) -> double {
  const auto piece_area = area_for_piece(piece_areas, piece);
  double cost = 0.0;
  if (insert_pos > 0U) {
    cost += std::abs(piece_area -
                     area_for_piece(piece_areas, order[insert_pos - 1U]));
  }
  if (insert_pos < order.size()) {
    cost +=
        std::abs(piece_area - area_for_piece(piece_areas, order[insert_pos]));
  }
  cost += 0.01 * static_cast<double>(insert_pos);
  return cost;
}

struct InsertionChoice {
  std::size_t insert_pos{0};
  double best_cost{0.0};
  double second_cost{0.0};
};

[[nodiscard]] auto best_insertion_choice(std::span<const std::size_t> order,
                                         const std::size_t piece,
                                         std::span<const double> piece_areas)
    -> InsertionChoice {
  InsertionChoice choice{};
  choice.best_cost = std::numeric_limits<double>::infinity();
  choice.second_cost = std::numeric_limits<double>::infinity();
  for (std::size_t insert_pos = 0; insert_pos <= order.size(); ++insert_pos) {
    const auto cost = insertion_cost(order, piece, piece_areas, insert_pos);
    if (cost < choice.best_cost) {
      choice.second_cost = choice.best_cost;
      choice.best_cost = cost;
      choice.insert_pos = insert_pos;
    } else if (cost < choice.second_cost) {
      choice.second_cost = cost;
    }
  }
  if (!std::isfinite(choice.second_cost)) {
    choice.second_cost = choice.best_cost;
  }
  return choice;
}

void insert_removed_random(
    std::vector<std::size_t> &order,
    std::vector<std::optional<geom::RotationIndex>> &forced_rotations,
    std::span<const RemovedPiece> removed, runtime::DeterministicRng &rng) {
  for (const auto &piece : removed) {
    const auto insert_pos = rng.uniform_index(order.size() + 1U);
    order.insert(order.begin() + static_cast<std::ptrdiff_t>(insert_pos),
                 piece.piece);
    forced_rotations.insert(forced_rotations.begin() +
                                static_cast<std::ptrdiff_t>(insert_pos),
                            piece.forced_rotation);
  }
}

void insert_removed_regret(
    std::vector<std::size_t> &order,
    std::vector<std::optional<geom::RotationIndex>> &forced_rotations,
    std::vector<RemovedPiece> removed, std::span<const double> piece_areas) {
  while (!removed.empty()) {
    std::size_t best_index = 0U;
    InsertionChoice best_choice =
        best_insertion_choice(order, removed.front().piece, piece_areas);
    double best_regret = best_choice.second_cost - best_choice.best_cost;

    for (std::size_t index = 1; index < removed.size(); ++index) {
      const auto choice =
          best_insertion_choice(order, removed[index].piece, piece_areas);
      const auto regret = choice.second_cost - choice.best_cost;
      if (regret > best_regret ||
          (regret == best_regret && choice.best_cost < best_choice.best_cost)) {
        best_index = index;
        best_choice = choice;
        best_regret = regret;
      }
    }

    const auto piece = removed[best_index];
    order.insert(order.begin() +
                     static_cast<std::ptrdiff_t>(best_choice.insert_pos),
                 piece.piece);
    forced_rotations.insert(
        forced_rotations.begin() +
            static_cast<std::ptrdiff_t>(best_choice.insert_pos),
        piece.forced_rotation);
    removed.erase(removed.begin() + static_cast<std::ptrdiff_t>(best_index));
  }
}

[[nodiscard]] auto destroy_and_repair(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    std::span<const double> piece_areas, runtime::DeterministicRng &rng,
    const std::size_t destroy_count, const DestroyStrategy destroy_strategy,
    const bool use_regret_repair, const NeighborhoodSearch op)
    -> NeighborhoodMove {
  NeighborhoodMove move;
  move.op = op;
  move.order.assign(order.begin(), order.end());
  move.forced_rotations.assign(forced_rotations.begin(),
                               forced_rotations.end());
  if (move.forced_rotations.size() != move.order.size()) {
    move.forced_rotations.assign(move.order.size(), std::nullopt);
  }
  move.destroy_count = clamp_destroy_count(order.size(), destroy_count);
  if (move.destroy_count == 0U) {
    return move;
  }

  const auto removal_positions = select_removal_positions(
      order, piece_areas, rng, move.destroy_count, destroy_strategy);
  std::vector<RemovedPiece> removed;
  removed.reserve(move.destroy_count);
  for (auto it = removal_positions.rbegin(); it != removal_positions.rend();
       ++it) {
    removed.push_back({
        .piece = move.order[*it],
        .forced_rotation = move.forced_rotations[*it],
    });
    move.order.erase(move.order.begin() + static_cast<std::ptrdiff_t>(*it));
    move.forced_rotations.erase(move.forced_rotations.begin() +
                                static_cast<std::ptrdiff_t>(*it));
  }

  if (destroy_strategy == DestroyStrategy::area) {
    std::stable_sort(removed.begin(), removed.end(),
                     [&](const RemovedPiece &lhs, const RemovedPiece &rhs) {
                       if (area_for_piece(piece_areas, lhs.piece) !=
                           area_for_piece(piece_areas, rhs.piece)) {
                         return area_for_piece(piece_areas, lhs.piece) >
                                area_for_piece(piece_areas, rhs.piece);
                       }
                       return lhs.piece < rhs.piece;
                     });
  }

  if (use_regret_repair) {
    insert_removed_regret(move.order, move.forced_rotations, std::move(removed),
                          piece_areas);
  } else {
    insert_removed_random(move.order, move.forced_rotations, removed, rng);
  }

  move.changed =
      move.order != std::vector<std::size_t>(order.begin(), order.end());
  return move;
}

} // namespace

OrderEvaluator::OrderEvaluator(const NormalizedRequest &request,
                               const SolveControl &control,
                               const runtime::TimeBudget &time_budget,
                               const runtime::Stopwatch &stopwatch)
    : request_{request}, control_{control}, time_budget_{time_budget},
      stopwatch_{stopwatch},
      workspace_{control.workspace != nullptr ? control.workspace
                                              : &local_workspace_},
      piece_areas_{piece_areas_for(request)},
      piece_rotation_counts_{piece_rotation_counts_for(request)} {}

auto OrderEvaluator::evaluate(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    const std::uint64_t seed_bias) const -> SolutionPoolEntry {
  SolutionPoolEntry evaluated;
  evaluated.order.assign(order.begin(), order.end());
  evaluated.forced_rotations.assign(forced_rotations.begin(),
                                    forced_rotations.end());
  if (evaluated.forced_rotations.size() != evaluated.order.size()) {
    evaluated.forced_rotations.assign(evaluated.order.size(), std::nullopt);
  }

  pack::IrregularConstructivePacker packer;
  SolveControl decode_control{};
  decode_control.cancellation = control_.cancellation;
  decode_control.random_seed = seed_from_order(
      order, evaluated.forced_rotations, control_.random_seed, seed_bias);
  decode_control.workspace = workspace_;
  if (time_budget_.enabled()) {
    const auto remaining = time_budget_.remaining_milliseconds(stopwatch_);
    if (remaining == 0U) {
      evaluated.result.strategy = StrategyKind::sequential_backtrack;
      evaluated.result.total_parts = request_.expanded_pieces.size();
      evaluated.result.stop_reason = StopReason::time_limit_reached;
      return evaluated;
    }
    decode_control.time_limit_milliseconds = remaining;
  }

  const auto result_or = packer.solve(
      reordered_request_for(order, evaluated.forced_rotations, request_),
      decode_control);
  if (result_or.ok() && !constructive_layout_has_geometry_violation(
                            request_, result_or.value())) {
    evaluated.result = result_or.value();
    evaluated.metrics = metrics_for_layout(evaluated.result.layout);
    return evaluated;
  }

  evaluated.result.strategy = StrategyKind::sequential_backtrack;
  evaluated.result.total_parts = request_.expanded_pieces.size();
  evaluated.result.stop_reason = StopReason::invalid_request;
  return evaluated;
}

auto OrderEvaluator::interrupted() const -> bool {
  return control_.cancellation.stop_requested() ||
         time_budget_.expired(stopwatch_);
}

auto OrderEvaluator::make_budget(const std::size_t operations_completed) const
    -> BudgetState {
  return {
      .operation_limit_enabled = control_.operation_limit > 0U,
      .operation_limit = control_.operation_limit,
      .operations_completed = operations_completed,
      .time_limit_enabled = time_budget_.enabled(),
      .time_limit_milliseconds = time_budget_.limit_milliseconds(),
      .elapsed_milliseconds = stopwatch_.elapsed_milliseconds(),
      .cancellation_requested = control_.cancellation.stop_requested(),
  };
}

auto OrderEvaluator::piece_areas() const -> std::span<const double> {
  return piece_areas_;
}

auto OrderEvaluator::piece_rotation_counts() const
    -> std::span<const std::size_t> {
  return piece_rotation_counts_;
}

auto original_order(const NormalizedRequest &request)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> order(request.expanded_pieces.size());
  std::iota(order.begin(), order.end(), 0U);
  return order;
}

auto descending_area_order(std::span<const double> piece_areas)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> order(piece_areas.size());
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(order.begin(), order.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     if (piece_areas[lhs] != piece_areas[rhs]) {
                       return piece_areas[lhs] > piece_areas[rhs];
                     }
                     return lhs < rhs;
                   });
  return order;
}

auto reverse_order(const NormalizedRequest &request)
    -> std::vector<std::size_t> {
  auto order = original_order(request);
  std::reverse(order.begin(), order.end());
  return order;
}

auto random_order(const std::size_t piece_count, runtime::DeterministicRng &rng)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> order(piece_count);
  std::iota(order.begin(), order.end(), 0U);
  rng.shuffle(order);
  return order;
}

auto original_forced_rotations(const NormalizedRequest &request)
    -> std::vector<std::optional<geom::RotationIndex>> {
  if (request.forced_rotations.size() == request.expanded_pieces.size()) {
    return request.forced_rotations;
  }
  return std::vector<std::optional<geom::RotationIndex>>(
      request.expanded_pieces.size(), std::nullopt);
}

auto propose_move(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    const NormalizedRequest &request, std::span<const double> piece_areas,
    std::span<const std::size_t> piece_rotation_counts,
    runtime::DeterministicRng &rng, const NeighborhoodSearch op,
    const std::size_t intensity) -> NeighborhoodMove {
  NeighborhoodMove move;
  move.op = op;
  move.order.assign(order.begin(), order.end());
  move.forced_rotations.assign(forced_rotations.begin(),
                               forced_rotations.end());
  if (move.forced_rotations.size() != move.order.size()) {
    move.forced_rotations.assign(move.order.size(), std::nullopt);
  }
  if (order.size() < 2U) {
    return move;
  }

  switch (op) {
  case NeighborhoodSearch::adjacent_swap: {
    const auto lhs = rng.uniform_index(order.size() - 1U);
    move.primary_index = lhs;
    move.secondary_index = lhs + 1U;
    move.changed =
        swap_positions(move.order, move.forced_rotations, lhs, lhs + 1U);
    break;
  }
  case NeighborhoodSearch::random_swap: {
    const auto lhs = rng.uniform_index(order.size());
    auto rhs = rng.uniform_index(order.size());
    if (lhs == rhs) {
      rhs = (rhs + 1U) % order.size();
    }
    move.primary_index = lhs;
    move.secondary_index = rhs;
    move.changed = swap_positions(move.order, move.forced_rotations, lhs, rhs);
    break;
  }
  case NeighborhoodSearch::relocate: {
    const auto from = rng.uniform_index(order.size());
    auto to = rng.uniform_index(order.size());
    if (from == to) {
      to = (to + 1U) % order.size();
    }
    const auto value = move.order[from];
    const auto forced_rotation = move.forced_rotations[from];
    move.order.erase(move.order.begin() + static_cast<std::ptrdiff_t>(from));
    move.forced_rotations.erase(move.forced_rotations.begin() +
                                static_cast<std::ptrdiff_t>(from));
    if (to > from) {
      --to;
    }
    move.order.insert(move.order.begin() + static_cast<std::ptrdiff_t>(to),
                      value);
    move.forced_rotations.insert(move.forced_rotations.begin() +
                                     static_cast<std::ptrdiff_t>(to),
                                 forced_rotation);
    move.primary_index = from;
    move.secondary_index = to;
    move.changed =
        move.order != std::vector<std::size_t>(order.begin(), order.end());
    break;
  }
  case NeighborhoodSearch::inversion: {
    auto lhs = rng.uniform_index(order.size());
    auto rhs = rng.uniform_index(order.size());
    if (lhs > rhs) {
      std::swap(lhs, rhs);
    }
    if (lhs == rhs) {
      rhs = std::min(order.size() - 1U, rhs + 1U);
    }
    std::reverse(move.order.begin() + static_cast<std::ptrdiff_t>(lhs),
                 move.order.begin() + static_cast<std::ptrdiff_t>(rhs + 1U));
    std::reverse(
        move.forced_rotations.begin() + static_cast<std::ptrdiff_t>(lhs),
        move.forced_rotations.begin() + static_cast<std::ptrdiff_t>(rhs + 1U));
    move.primary_index = lhs;
    move.secondary_index = rhs;
    move.changed = lhs != rhs;
    break;
  }
  case NeighborhoodSearch::large_item_swap: {
    const auto disrupted =
        disrupt_large_items(order, request, piece_areas, rng);
    move.order = disrupted.order;
    move.forced_rotations.assign(forced_rotations.begin(),
                                 forced_rotations.end());
    if (move.forced_rotations.size() != move.order.size()) {
      move.forced_rotations.assign(move.order.size(), std::nullopt);
    }
    if (disrupted.applied) {
      swap_positions(move.order, move.forced_rotations,
                     disrupted.first_position, disrupted.second_position);
      move.order = disrupted.order;
    }
    move.primary_index = disrupted.first_position;
    move.secondary_index = disrupted.second_position;
    move.changed = disrupted.applied;
    break;
  }
  case NeighborhoodSearch::rotation_change: {
    std::vector<std::size_t> rotatable_positions;
    rotatable_positions.reserve(order.size());
    for (std::size_t index = 0; index < order.size(); ++index) {
      const auto piece_index = order[index];
      if (piece_index < piece_rotation_counts.size() &&
          piece_rotation_counts[piece_index] > 1U) {
        rotatable_positions.push_back(index);
      }
    }
    if (rotatable_positions.empty()) {
      break;
    }
    const auto position =
        rotatable_positions[rng.uniform_index(rotatable_positions.size())];
    const auto piece_index = order[position];
    const auto rotation_count = piece_rotation_counts[piece_index];
    const auto current_rotation = move.forced_rotations[position].has_value()
                                      ? move.forced_rotations[position]->value
                                      : 0U;
    const auto delta = 1U + rng.uniform_index(rotation_count - 1U);
    const auto next_rotation =
        static_cast<std::uint16_t>((current_rotation + delta) % rotation_count);
    move.forced_rotations[position] = geom::RotationIndex{next_rotation};
    move.primary_index = position;
    move.secondary_index = next_rotation;
    move.changed =
        !forced_rotations.empty()
            ? move.forced_rotations !=
                  std::vector<std::optional<geom::RotationIndex>>(
                      forced_rotations.begin(), forced_rotations.end())
            : true;
    break;
  }
  case NeighborhoodSearch::random_destroy_repair:
    move = destroy_and_repair(order, forced_rotations, piece_areas, rng,
                              intensity, DestroyStrategy::random, false, op);
    break;
  case NeighborhoodSearch::area_destroy_repair:
    move = destroy_and_repair(order, forced_rotations, piece_areas, rng,
                              intensity, DestroyStrategy::area, false, op);
    break;
  case NeighborhoodSearch::related_destroy_repair:
    move = destroy_and_repair(order, forced_rotations, piece_areas, rng,
                              intensity, DestroyStrategy::related, false, op);
    break;
  case NeighborhoodSearch::cluster_destroy_repair:
    move = destroy_and_repair(order, forced_rotations, piece_areas, rng,
                              intensity, DestroyStrategy::cluster, false, op);
    break;
  case NeighborhoodSearch::regret_destroy_repair:
    move = destroy_and_repair(order, forced_rotations, piece_areas, rng,
                              intensity, DestroyStrategy::area, true, op);
    break;
  }

  if (!move.changed && order.size() >= 2U) {
    move.op = NeighborhoodSearch::random_swap;
    move.order.assign(order.begin(), order.end());
    move.forced_rotations.assign(forced_rotations.begin(),
                                 forced_rotations.end());
    if (move.forced_rotations.size() != move.order.size()) {
      move.forced_rotations.assign(move.order.size(), std::nullopt);
    }
    const auto lhs = rng.uniform_index(order.size());
    auto rhs = rng.uniform_index(order.size());
    if (lhs == rhs) {
      rhs = (rhs + 1U) % order.size();
    }
    move.primary_index = lhs;
    move.secondary_index = rhs;
    move.changed = swap_positions(move.order, move.forced_rotations, lhs, rhs);
  }

  return move;
}

auto random_operator(runtime::DeterministicRng &rng,
                     const bool include_destroy_repair) -> NeighborhoodSearch {
  const std::size_t count = include_destroy_repair ? 11U : 6U;
  return static_cast<NeighborhoodSearch>(rng.uniform_index(count));
}

auto all_alns_operators() -> std::vector<NeighborhoodSearch> {
  return {
      NeighborhoodSearch::random_destroy_repair,
      NeighborhoodSearch::area_destroy_repair,
      NeighborhoodSearch::related_destroy_repair,
      NeighborhoodSearch::cluster_destroy_repair,
      NeighborhoodSearch::regret_destroy_repair,
      NeighborhoodSearch::large_item_swap,
      NeighborhoodSearch::rotation_change,
      NeighborhoodSearch::relocate,
  };
}

// Maximisation objective: larger is better. The lexicographic
// priority is encoded as huge weights so adding the lower terms
// can never invert the order set by a higher-priority delta:
//   placed_parts (10¹²) ≫ bin_count (10⁹) ≫ strip_length (10⁴) ≫
//   utilization (10³). All call sites compute `delta = candidate -
//   current` and accept on positive delta (see SA Metropolis,
//   LAHC history compare, etc.).
auto objective_score(const LayoutMetrics &metrics) -> double {
  return static_cast<double>(metrics.placed_parts) * 1'000'000'000'000.0 -
         static_cast<double>(metrics.bin_count) * 1'000'000'000.0 -
         metrics.strip_length * 10'000.0 + metrics.utilization * 1'000.0;
}

auto primary_metrics_preserved(const LayoutMetrics &candidate,
                               const LayoutMetrics &reference) -> bool {
  if (candidate.placed_parts != reference.placed_parts) {
    return candidate.placed_parts > reference.placed_parts;
  }
  if (candidate.bin_count != reference.bin_count) {
    return candidate.bin_count < reference.bin_count;
  }
  return true;
}

auto within_record_window(const LayoutMetrics &candidate,
                          const LayoutMetrics &best,
                          const double tolerance_ratio) -> bool {
  if (!primary_metrics_preserved(candidate, best)) {
    return false;
  }
  if (better_metrics(candidate, best)) {
    return true;
  }

  const double ratio = std::max(0.0, tolerance_ratio);
  if (best.strip_length > 0.0 &&
      candidate.strip_length > best.strip_length * (1.0 + ratio)) {
    return false;
  }
  return candidate.utilization + (ratio * 0.5) >= best.utilization;
}

auto operator_label(const NeighborhoodSearch op) -> std::string_view {
  switch (op) {
  case NeighborhoodSearch::adjacent_swap:
    return "adjacent-swap";
  case NeighborhoodSearch::random_swap:
    return "random-swap";
  case NeighborhoodSearch::relocate:
    return "relocate";
  case NeighborhoodSearch::inversion:
    return "inversion";
  case NeighborhoodSearch::large_item_swap:
    return "large-item-swap";
  case NeighborhoodSearch::rotation_change:
    return "rotation-change";
  case NeighborhoodSearch::random_destroy_repair:
    return "random-destroy-repair";
  case NeighborhoodSearch::area_destroy_repair:
    return "area-destroy-repair";
  case NeighborhoodSearch::related_destroy_repair:
    return "related-destroy-repair";
  case NeighborhoodSearch::cluster_destroy_repair:
    return "cluster-destroy-repair";
  case NeighborhoodSearch::regret_destroy_repair:
    return "regret-destroy-repair";
  }
  return "unknown";
}

} // namespace shiny::nesting::search::detail
