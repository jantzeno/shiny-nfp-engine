#include "request.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "geometry/normalize.hpp"
#include "geometry/transform.hpp"
#include "geometry/validity.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting {
namespace detail {

template <typename T> auto hash_value(std::uint64_t &hash, const T &value) -> void {
  runtime::hash::fnv1a_mix_value(hash, value);
}

[[nodiscard]] auto derive_expanded_id(const std::uint32_t base_id,
                                      const std::uint32_t ordinal,
                                      std::unordered_set<std::uint32_t> &used_ids)
    -> std::uint32_t {
  if (ordinal == 0U && !used_ids.contains(base_id)) {
    used_ids.insert(base_id);
    return base_id;
  }

  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  hash_value(hash, base_id);
  hash_value(hash, ordinal);
  auto candidate = static_cast<std::uint32_t>(hash & 0xffffffffu);
  while (used_ids.contains(candidate)) {
    hash_value(hash, candidate);
    candidate = static_cast<std::uint32_t>(hash & 0xffffffffu);
  }
  used_ids.insert(candidate);
  return candidate;
}

[[nodiscard]] auto rotation_set_is_valid(const geom::DiscreteRotationSet &rotations)
    -> bool {
  place::PlacementConfig config;
  config.allowed_rotations = rotations;
  return config.is_valid();
}

[[nodiscard]] auto rotation_set_is_subset(const geom::DiscreteRotationSet &subset,
                                          const geom::DiscreteRotationSet &superset)
    -> bool {
  const auto subset_angles = geom::materialize_rotations(subset);
  const auto superset_angles = geom::materialize_rotations(superset);
  return std::all_of(subset_angles.begin(), subset_angles.end(),
                     [&](const double angle) {
                       return std::find(superset_angles.begin(),
                                        superset_angles.end(),
                                        angle) != superset_angles.end();
                     });
}

[[nodiscard]] auto normalize_piece_polygon(const PieceRequest &piece,
                                          const PreprocessPolicy &policy)
    -> geom::PolygonWithHoles {
  auto polygon = geom::normalize_polygon(piece.polygon);
  if (policy.simplify_epsilon > 0.0) {
    polygon = poly::simplify_polygon_douglas_peucker(polygon,
                                                     policy.simplify_epsilon);
  } else {
    polygon = poly::simplify_polygon(polygon);
  }

  if (policy.normalize_piece_origins) {
    const auto bounds = geom::compute_bounds(polygon);
    polygon = geom::translate(
        polygon, {.x = -bounds.min.x, .y = -bounds.min.y});
  }

  return geom::normalize_polygon(polygon);
}

[[nodiscard]] auto normalize_bin_polygon(const BinRequest &bin,
                                         const PreprocessPolicy &policy)
    -> geom::PolygonWithHoles {
  auto polygon = geom::normalize_polygon(bin.polygon);
  if (policy.simplify_epsilon > 0.0) {
    polygon = poly::simplify_polygon_douglas_peucker(polygon,
                                                     policy.simplify_epsilon);
  } else {
    polygon = poly::simplify_polygon(polygon);
  }
  return geom::normalize_polygon(polygon);
}

[[nodiscard]] auto sort_unique_ids(std::vector<std::uint32_t> ids)
    -> std::vector<std::uint32_t> {
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

[[nodiscard]] auto production_optimizer_is_valid(
    const ProductionOptimizerKind optimizer) -> bool {
  switch (optimizer) {
  case ProductionOptimizerKind::brkga:
  case ProductionOptimizerKind::simulated_annealing:
  case ProductionOptimizerKind::alns:
  case ProductionOptimizerKind::gdrr:
  case ProductionOptimizerKind::lahc:
    return true;
  }
  return false;
}

[[nodiscard]] auto strategy_kind_is_valid(const StrategyKind strategy) -> bool {
  switch (strategy) {
  case StrategyKind::bounding_box:
  case StrategyKind::sequential_backtrack:
  case StrategyKind::metaheuristic_search:
  case StrategyKind::simulated_annealing:
  case StrategyKind::alns:
  case StrategyKind::gdrr:
  case StrategyKind::lahc:
    return true;
  }
  return false;
}

void synchronize_strategy_configs(ExecutionPolicy &execution) {
  switch (execution.strategy) {
  case StrategyKind::simulated_annealing:
    if (execution.strategy_config.get_if<SAConfig>(StrategyKind::simulated_annealing) ==
        nullptr) {
      execution.strategy_config = StrategyConfig::make(
          StrategyKind::simulated_annealing, execution.simulated_annealing);
    }
    break;
  case StrategyKind::alns:
    if (execution.strategy_config.get_if<ALNSConfig>(StrategyKind::alns) == nullptr) {
      execution.strategy_config =
          StrategyConfig::make(StrategyKind::alns, execution.alns);
    }
    break;
  case StrategyKind::gdrr:
    if (execution.strategy_config.get_if<GDRRConfig>(StrategyKind::gdrr) == nullptr) {
      execution.strategy_config =
          StrategyConfig::make(StrategyKind::gdrr, execution.gdrr);
    }
    break;
  case StrategyKind::lahc:
    if (execution.strategy_config.get_if<LAHCConfig>(StrategyKind::lahc) == nullptr) {
      execution.strategy_config =
          StrategyConfig::make(StrategyKind::lahc, execution.lahc);
    }
    break;
  case StrategyKind::bounding_box:
  case StrategyKind::sequential_backtrack:
  case StrategyKind::metaheuristic_search:
    break;
  }

  switch (execution.production_optimizer) {
  case ProductionOptimizerKind::brkga:
    if (execution.production_strategy_config.get_if<ProductionSearchConfig>(
            ProductionOptimizerKind::brkga) == nullptr) {
      execution.production_strategy_config = ProductionStrategyConfig::make(
          ProductionOptimizerKind::brkga, execution.production);
    }
    break;
  case ProductionOptimizerKind::simulated_annealing:
    if (execution.production_strategy_config.get_if<SAConfig>(
            ProductionOptimizerKind::simulated_annealing) == nullptr) {
      execution.production_strategy_config = ProductionStrategyConfig::make(
          ProductionOptimizerKind::simulated_annealing,
          execution.simulated_annealing);
    }
    break;
  case ProductionOptimizerKind::alns:
    if (execution.production_strategy_config.get_if<ALNSConfig>(
            ProductionOptimizerKind::alns) == nullptr) {
      execution.production_strategy_config = ProductionStrategyConfig::make(
          ProductionOptimizerKind::alns, execution.alns);
    }
    break;
  case ProductionOptimizerKind::gdrr:
    if (execution.production_strategy_config.get_if<GDRRConfig>(
            ProductionOptimizerKind::gdrr) == nullptr) {
      execution.production_strategy_config = ProductionStrategyConfig::make(
          ProductionOptimizerKind::gdrr, execution.gdrr);
    }
    break;
  case ProductionOptimizerKind::lahc:
    if (execution.production_strategy_config.get_if<LAHCConfig>(
            ProductionOptimizerKind::lahc) == nullptr) {
      execution.production_strategy_config = ProductionStrategyConfig::make(
          ProductionOptimizerKind::lahc, execution.lahc);
    }
    break;
  }
}

[[nodiscard]] auto active_strategy_config_is_valid(const ExecutionPolicy &execution)
    -> bool {
  switch (execution.strategy) {
  case StrategyKind::simulated_annealing:
    return resolve_primary_strategy_config(
               execution, StrategyKind::simulated_annealing,
               execution.simulated_annealing)
        .is_valid();
  case StrategyKind::alns:
    return resolve_primary_strategy_config(execution, StrategyKind::alns,
                                           execution.alns)
        .is_valid();
  case StrategyKind::gdrr:
    return resolve_primary_strategy_config(execution, StrategyKind::gdrr,
                                           execution.gdrr)
        .is_valid();
  case StrategyKind::lahc:
    return resolve_primary_strategy_config(execution, StrategyKind::lahc,
                                           execution.lahc)
        .is_valid();
  case StrategyKind::bounding_box:
  case StrategyKind::sequential_backtrack:
  case StrategyKind::metaheuristic_search:
    return true;
  }
  return false;
}

[[nodiscard]] auto active_production_strategy_config_is_valid(
    const ExecutionPolicy &execution) -> bool {
  switch (execution.production_optimizer) {
  case ProductionOptimizerKind::brkga:
    return resolve_production_strategy_config(
               execution, ProductionOptimizerKind::brkga, execution.production)
        .is_valid();
  case ProductionOptimizerKind::simulated_annealing:
    return resolve_production_strategy_config(
               execution, ProductionOptimizerKind::simulated_annealing,
               execution.simulated_annealing)
        .is_valid();
  case ProductionOptimizerKind::alns:
    return resolve_production_strategy_config(execution,
                                              ProductionOptimizerKind::alns,
                                              execution.alns)
        .is_valid();
  case ProductionOptimizerKind::gdrr:
    return resolve_production_strategy_config(execution,
                                              ProductionOptimizerKind::gdrr,
                                              execution.gdrr)
        .is_valid();
  case ProductionOptimizerKind::lahc:
    return resolve_production_strategy_config(execution,
                                              ProductionOptimizerKind::lahc,
                                              execution.lahc)
        .is_valid();
  }
  return false;
}

} // namespace detail

auto ProductionSearchConfig::is_valid() const -> bool {
  return population_size >= 2U && elite_count >= 1U &&
         elite_count < population_size && mutant_count < population_size &&
         elite_count + mutant_count < population_size &&
         max_iterations >= 1U && std::isfinite(elite_bias) && elite_bias > 0.0 &&
         elite_bias < 1.0 && std::isfinite(gls_weight_cap) &&
         gls_weight_cap >= 1.0;
}

auto SAConfig::is_valid() const -> bool {
  switch (cooling_schedule) {
  case CoolingScheduleKind::geometric:
  case CoolingScheduleKind::linear:
  case CoolingScheduleKind::adaptive:
  case CoolingScheduleKind::lundy_mees:
    break;
  default:
    return false;
  }

  return max_refinements >= 1U && restart_count >= 1U &&
         std::isfinite(initial_temperature) && initial_temperature > 0.0 &&
         std::isfinite(final_temperature) && final_temperature > 0.0 &&
         initial_temperature >= final_temperature && std::isfinite(lundy_beta) &&
         lundy_beta > 0.0 && plateau_window <= 1024U &&
         std::isfinite(reheating_factor) && reheating_factor >= 1.0 &&
         perturbation_swaps <= 32U;
}

auto ALNSConfig::is_valid() const -> bool {
  return max_refinements >= 1U && destroy_min_count >= 1U &&
         destroy_max_count >= destroy_min_count && destroy_max_count <= 64U &&
         std::isfinite(initial_acceptance_ratio) &&
         initial_acceptance_ratio >= 0.0 &&
         std::isfinite(final_acceptance_ratio) &&
         final_acceptance_ratio >= 0.0 &&
         initial_acceptance_ratio >= final_acceptance_ratio &&
         std::isfinite(reaction_factor) && reaction_factor > 0.0 &&
         reaction_factor <= 1.0 && std::isfinite(reward_improve) &&
         reward_improve >= 0.0 && std::isfinite(reward_accept) &&
         reward_accept >= 0.0 && std::isfinite(reward_reject) &&
         reward_reject >= 0.0 && segment_length >= 1U;
}

auto GDRRConfig::is_valid() const -> bool {
  return max_refinements >= 1U && std::isfinite(initial_goal_ratio) &&
         initial_goal_ratio > 0.0 && initial_goal_ratio <= 1.0 &&
         std::isfinite(goal_decay) && goal_decay > 0.0 && goal_decay < 1.0 &&
         ruin_swap_count <= 32U;
}

auto LAHCConfig::is_valid() const -> bool {
  return max_refinements >= 1U && history_length >= 1U &&
         history_length <= 4096U && plateau_limit <= 4096U &&
         perturbation_swaps <= 32U;
}

auto IrregularOptions::is_valid() const -> bool {
  if (max_candidate_points == 0U || max_candidate_points > 100'000U) {
    return false;
  }
  if (!std::isfinite(candidate_gaussian_sigma) ||
      candidate_gaussian_sigma <= 0.0) {
    return false;
  }
  if (max_backtrack_pieces > 8U || compaction_passes > 16U) {
    return false;
  }

  switch (candidate_strategy) {
  case CandidateStrategy::anchor_vertex:
  case CandidateStrategy::nfp_perfect:
  case CandidateStrategy::nfp_arrangement:
  case CandidateStrategy::nfp_hybrid:
    break;
  default:
    return false;
  }

  switch (piece_ordering) {
  case PieceOrdering::input:
  case PieceOrdering::largest_area_first:
  case PieceOrdering::hull_diameter_first:
  case PieceOrdering::priority:
    return true;
  }
  return false;
}

auto NestingRequest::is_valid() const -> bool {
  if (!std::isfinite(preprocess.simplify_epsilon) ||
      preprocess.simplify_epsilon < 0.0) {
    return false;
  }
  if (!std::isfinite(execution.part_spacing) || execution.part_spacing < 0.0) {
    return false;
  }
  if (!detail::strategy_kind_is_valid(execution.strategy) ||
      !detail::production_optimizer_is_valid(execution.production_optimizer) ||
      !detail::rotation_set_is_valid(execution.default_rotations) ||
      !execution.bounding_box.is_valid() ||
      !execution.deterministic_attempts.is_valid() ||
      !execution.irregular.is_valid() || !execution.production.is_valid() ||
      !execution.simulated_annealing.is_valid() || !execution.alns.is_valid() ||
      !execution.gdrr.is_valid() || !execution.lahc.is_valid() ||
      !detail::active_strategy_config_is_valid(execution) ||
      !detail::active_production_strategy_config_is_valid(execution)) {
    return false;
  }

  std::unordered_set<std::uint32_t> bin_ids;
  for (const auto &bin : bins) {
    if (bin.stock == 0U || !bin_ids.insert(bin.bin_id).second) {
      return false;
    }
    if (!std::all_of(bin.exclusion_zones.begin(), bin.exclusion_zones.end(),
                     [](const auto &zone) { return zone.is_valid(); })) {
      return false;
    }
  }

  std::unordered_set<std::uint32_t> piece_ids;
  for (const auto &piece : pieces) {
    if (piece.quantity == 0U || !piece_ids.insert(piece.piece_id).second) {
      return false;
    }
    if (piece.allowed_rotations.has_value() &&
        (!detail::rotation_set_is_valid(*piece.allowed_rotations) ||
         !detail::rotation_set_is_subset(*piece.allowed_rotations,
                                         execution.default_rotations))) {
      return false;
    }
  }

  return true;
}

auto normalize_request(const NestingRequest &request)
    -> util::StatusOr<NormalizedRequest> {
  if (!request.is_valid()) {
    return util::Status::invalid_input;
  }

  NormalizedRequest normalized;
  normalized.request.preprocess = request.preprocess;
  normalized.request.execution = request.execution;
  detail::synchronize_strategy_configs(normalized.request.execution);
  normalized.request.execution.selected_bin_ids =
      detail::sort_unique_ids(normalized.request.execution.selected_bin_ids);

  const auto selected_bin_ids = normalized.request.execution.selected_bin_ids;
  std::unordered_set<std::uint32_t> selected_lookup(selected_bin_ids.begin(),
                                                    selected_bin_ids.end());

  for (const auto &bin : request.bins) {
    if (!selected_lookup.empty() && !selected_lookup.contains(bin.bin_id)) {
      continue;
    }

    if (request.preprocess.discard_empty_bins && bin.stock == 0U) {
      continue;
    }

    BinRequest normalized_bin = bin;
    normalized_bin.polygon = detail::normalize_bin_polygon(bin, request.preprocess);
    if (geom::validate_polygon(normalized_bin.polygon).is_valid() == false) {
      return util::Status::invalid_input;
    }
    if (normalized_bin.geometry_revision == 0U) {
      normalized_bin.geometry_revision =
          geom::polygon_revision(normalized_bin.polygon);
    }
    normalized.request.bins.push_back(std::move(normalized_bin));
  }

  if (normalized.request.bins.empty()) {
    return util::Status::invalid_input;
  }

  for (const auto &piece : request.pieces) {
    PieceRequest normalized_piece = piece;
    normalized_piece.polygon =
        detail::normalize_piece_polygon(piece, request.preprocess);
    normalized_piece.allowed_bin_ids =
        detail::sort_unique_ids(normalized_piece.allowed_bin_ids);

    if (geom::validate_polygon(normalized_piece.polygon).is_valid() == false) {
      return util::Status::invalid_input;
    }
    if (normalized_piece.geometry_revision == 0U) {
      normalized_piece.geometry_revision =
          geom::polygon_revision(normalized_piece.polygon);
    }
    normalized.request.pieces.push_back(std::move(normalized_piece));
  }

  if (normalized.request.pieces.empty()) {
    return util::Status::invalid_input;
  }

  std::unordered_set<std::uint32_t> used_bin_ids;
  for (const auto &bin : normalized.request.bins) {
    for (std::uint32_t stock_index = 0; stock_index < bin.stock; ++stock_index) {
      normalized.expanded_bins.push_back({
          .source_bin_id = bin.bin_id,
          .expanded_bin_id =
              detail::derive_expanded_id(bin.bin_id, stock_index, used_bin_ids),
          .stock_index = stock_index,
      });
    }
  }

  std::unordered_set<std::uint32_t> used_piece_ids;
  for (const auto &piece : normalized.request.pieces) {
    for (std::uint32_t instance_index = 0; instance_index < piece.quantity;
         ++instance_index) {
      normalized.expanded_pieces.push_back({
          .source_piece_id = piece.piece_id,
          .expanded_piece_id = detail::derive_expanded_id(piece.piece_id,
                                                          instance_index,
                                                          used_piece_ids),
          .instance_index = instance_index,
      });
      normalized.forced_rotations.push_back(std::nullopt);
    }
  }

  return normalized;
}

auto to_bounding_box_decoder_request(const NormalizedRequest &request)
    -> util::StatusOr<pack::DecoderRequest> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  pack::DecoderRequest decoder_request;
  decoder_request.policy = request.request.execution.placement_policy;
  decoder_request.config.placement.part_clearance =
      request.request.execution.part_spacing;
  decoder_request.config.placement.allowed_rotations =
      request.request.execution.default_rotations;
  decoder_request.config.placement.enable_part_in_part_placement =
      request.request.execution.enable_part_in_part_placement;
  decoder_request.config.placement.explore_concave_candidates =
      request.request.execution.explore_concave_candidates;
  decoder_request.config.enable_hole_first_placement =
      request.request.execution.enable_part_in_part_placement;
  decoder_request.config.bounding_box = request.request.execution.bounding_box;
  decoder_request.config.deterministic_attempts =
      request.request.execution.deterministic_attempts;

  std::unordered_map<std::uint32_t, std::vector<std::uint32_t>>
      expanded_bin_ids_by_source;
  for (const auto &expanded_bin : request.expanded_bins) {
    const auto source_it =
        std::find_if(request.request.bins.begin(), request.request.bins.end(),
                     [&](const auto &bin) {
                       return bin.bin_id == expanded_bin.source_bin_id;
                     });
    if (source_it == request.request.bins.end()) {
      return util::Status::invalid_input;
    }

    decoder_request.bins.push_back({
        .bin_id = expanded_bin.expanded_bin_id,
        .polygon = source_it->polygon,
        .geometry_revision = source_it->geometry_revision,
        .start_corner = source_it->start_corner,
    });
    expanded_bin_ids_by_source[expanded_bin.source_bin_id].push_back(
        expanded_bin.expanded_bin_id);

    for (const auto &zone : source_it->exclusion_zones) {
      auto expanded_zone = zone;
      expanded_zone.bin_id = expanded_bin.expanded_bin_id;
      decoder_request.config.placement.exclusion_zones.push_back(
          std::move(expanded_zone));
    }
  }

  for (const auto &expanded_piece : request.expanded_pieces) {
    const auto source_it =
        std::find_if(request.request.pieces.begin(), request.request.pieces.end(),
                     [&](const auto &piece) {
                       return piece.piece_id == expanded_piece.source_piece_id;
                     });
    if (source_it == request.request.pieces.end()) {
      return util::Status::invalid_input;
    }

    pack::PieceInput piece{
        .piece_id = expanded_piece.expanded_piece_id,
        .polygon = source_it->polygon,
        .geometry_revision = source_it->geometry_revision,
        .allow_mirror = source_it->allow_mirror,
        .allowed_rotations = source_it->allowed_rotations,
        .grain_compatibility = source_it->grain_compatibility,
        .restricted_to_allowed_bins = !source_it->allowed_bin_ids.empty(),
    };

    if (!source_it->allowed_bin_ids.empty()) {
      for (const auto source_bin_id : source_it->allowed_bin_ids) {
        const auto bins_it = expanded_bin_ids_by_source.find(source_bin_id);
        if (bins_it == expanded_bin_ids_by_source.end()) {
          continue;
        }
        piece.allowed_bin_ids.insert(piece.allowed_bin_ids.end(),
                                     bins_it->second.begin(), bins_it->second.end());
      }
      piece.allowed_bin_ids = detail::sort_unique_ids(piece.allowed_bin_ids);
    }

    decoder_request.pieces.push_back(std::move(piece));
  }

  if (!decoder_request.config.is_valid()) {
    return util::Status::invalid_input;
  }
  return decoder_request;
}

} // namespace shiny::nesting
