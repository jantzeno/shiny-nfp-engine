#include "internal/request_normalization.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "api/profiles.hpp"
#include "geometry/operations/simplify.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/queries/validity.hpp"
#include "geometry/transforms/transform.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting {
namespace detail {

template <typename T>
auto hash_value(std::uint64_t &hash, const T &value) -> void {
  runtime::hash::fnv1a_mix_value(hash, value);
}

[[nodiscard]] auto
derive_expanded_id(const std::uint32_t base_id, const std::uint32_t ordinal,
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

[[nodiscard]] auto
rotation_set_is_valid(const geom::DiscreteRotationSet &rotations) -> bool {
  place::PlacementConfig config;
  config.allowed_rotations = rotations;
  return config.is_valid();
}

[[nodiscard]] auto
rotation_set_is_subset(const geom::DiscreteRotationSet &subset,
                       const geom::DiscreteRotationSet &superset) -> bool {
  const auto subset_angles = geom::materialize_rotations(subset);
  const auto superset_angles = geom::materialize_rotations(superset);
  return std::ranges::all_of(subset_angles, [&](const double angle) {
    return std::ranges::contains(superset_angles, angle);
  });
}

[[nodiscard]] auto normalize_piece_polygon(const PieceRequest &piece,
                                           const PreprocessPolicy &policy)
    -> geom::PolygonWithHoles {
  auto polygon = geom::normalize_polygon(piece.polygon);
  if (policy.simplify_epsilon > 0.0) {
    polygon = geom::simplify_polygon_douglas_peucker(polygon,
                                                     policy.simplify_epsilon);
  } else {
    polygon = geom::simplify_polygon(polygon);
  }

  if (policy.normalize_piece_origins) {
    const auto bounds = geom::compute_bounds(polygon);
    polygon = geom::translate(polygon,
                              geom::Vector2{-bounds.min.x(), -bounds.min.y()});
  }

  return geom::normalize_polygon(polygon);
}

[[nodiscard]] auto normalize_bin_polygon(const BinRequest &bin,
                                         const PreprocessPolicy &policy)
    -> geom::PolygonWithHoles {
  auto polygon = geom::normalize_polygon(bin.polygon);
  if (policy.simplify_epsilon > 0.0) {
    polygon = geom::simplify_polygon_douglas_peucker(polygon,
                                                     policy.simplify_epsilon);
  } else {
    polygon = geom::simplify_polygon(polygon);
  }
  return geom::normalize_polygon(polygon);
}

[[nodiscard]] auto sort_unique_ids(std::vector<std::uint32_t> ids)
    -> std::vector<std::uint32_t> {
  std::ranges::sort(ids);
  ids.erase(std::ranges::unique(ids).begin(), ids.end());
  return ids;
}

[[nodiscard]] auto
production_optimizer_is_valid(const ProductionOptimizerKind optimizer) -> bool {
  switch (optimizer) {
  case ProductionOptimizerKind::brkga:
    return true;
  }
  return false;
}

[[nodiscard]] auto objective_mode_is_valid(const ObjectiveMode mode) -> bool {
  switch (mode) {
  case ObjectiveMode::placement_count:
  case ObjectiveMode::maximize_value:
    return true;
  }
  return false;
}

[[nodiscard]] auto strategy_kind_is_valid(const StrategyKind strategy) -> bool {
  switch (strategy) {
  case StrategyKind::bounding_box:
  case StrategyKind::metaheuristic_search:
    return true;
  }
  return false;
}

void synchronize_strategy_configs(ExecutionPolicy &execution) {
  switch (execution.strategy) {
  case StrategyKind::bounding_box:
  case StrategyKind::metaheuristic_search:
    break;
  }

  switch (execution.production_optimizer) {
  case ProductionOptimizerKind::brkga:
    if (!has_production_strategy_config<ProductionSearchConfig>(
            execution, ProductionOptimizerKind::brkga)) {
      set_production_strategy_config(execution, ProductionOptimizerKind::brkga,
                                     execution.production);
    }
    break;
  }
}

[[nodiscard]] auto
active_strategy_config_is_valid(const ExecutionPolicy &execution) -> bool {
  switch (execution.strategy) {
  case StrategyKind::bounding_box:
  case StrategyKind::metaheuristic_search:
    return true;
  }
  return false;
}

[[nodiscard]] auto
active_production_strategy_config_is_valid(const ExecutionPolicy &execution)
    -> bool {
  switch (execution.production_optimizer) {
  case ProductionOptimizerKind::brkga:
    return resolve_production_strategy_config(
               execution, ProductionOptimizerKind::brkga, execution.production)
        .is_valid();
  }
  return false;
}

} // namespace detail

auto ProfileRequest::validate() const -> std::expected<void, util::Status> {
  if (!api::profile_is_valid(profile)) {
    return std::unexpected(util::Status::invalid_input);
  }
  if (api::profile_requires_time_limit(profile) &&
      (!time_limit_milliseconds.has_value() ||
       *time_limit_milliseconds == 0U)) {
    return std::unexpected(util::Status::invalid_input);
  }
  return {};
}

auto ProfileRequest::is_valid() const -> bool {
  if (!validate()) {
    return false;
  }
  return to_nesting_request(*this).has_value();
}

auto ProductionSearchConfig::is_valid() const -> bool {
  return population_size >= 2U && elite_count >= 1U &&
         elite_count < population_size && mutant_count < population_size &&
         elite_count + mutant_count < population_size && max_iterations >= 1U &&
         std::isfinite(elite_bias) && elite_bias > 0.0 && elite_bias < 1.0 &&
         std::isfinite(strip_exploration_ratio) &&
         strip_exploration_ratio > 0.0 && strip_exploration_ratio < 1.0 &&
         std::isfinite(strip_exploration_shrink_max_ratio) &&
         std::isfinite(strip_exploration_shrink_min_ratio) &&
         strip_exploration_shrink_max_ratio >=
             strip_exploration_shrink_min_ratio &&
         strip_exploration_shrink_min_ratio >= 0.0 &&
         std::isfinite(strip_compression_shrink_max_ratio) &&
         std::isfinite(strip_compression_shrink_min_ratio) &&
         strip_compression_shrink_max_ratio >=
             strip_compression_shrink_min_ratio &&
         strip_compression_shrink_min_ratio >= 0.0 &&
         infeasible_pool_capacity <= population_size &&
         infeasible_rollback_after <= max_iterations &&
         separator_worker_count >= 1U && separator_max_iterations >= 1U &&
         separator_iter_no_improvement_limit >= 1U &&
         separator_strike_limit >= 1U && separator_global_samples >= 1U &&
         separator_focused_samples >= 1U &&
         separator_coordinate_descent_iterations >= 1U &&
         std::isfinite(gls_weight_cap) && gls_weight_cap >= 1.0;
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
      !detail::objective_mode_is_valid(execution.objective_mode) ||
      !detail::rotation_set_is_valid(execution.default_rotations) ||
      !execution.bounding_box.is_valid() ||
      !execution.deterministic_attempts.is_valid() ||
      !execution.irregular.is_valid() || !execution.production.is_valid() ||
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
    if (!std::isfinite(piece.value) || piece.value < 0.0) {
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
    -> std::expected<NormalizedRequest, util::Status> {
  if (!request.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
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
    normalized_bin.polygon =
        detail::normalize_bin_polygon(bin, request.preprocess);
    if (geom::validate_polygon(normalized_bin.polygon).is_valid() == false) {
      return std::unexpected(util::Status::invalid_input);
    }
    if (normalized_bin.geometry_revision == 0U) {
      normalized_bin.geometry_revision =
          geom::polygon_revision(normalized_bin.polygon);
    }
    normalized.request.bins.push_back(std::move(normalized_bin));
  }

  if (normalized.request.bins.empty()) {
    return std::unexpected(util::Status::invalid_input);
  }

  for (const auto &piece : request.pieces) {
    PieceRequest normalized_piece = piece;
    normalized_piece.polygon =
        detail::normalize_piece_polygon(piece, request.preprocess);
    normalized_piece.allowed_bin_ids =
        detail::sort_unique_ids(normalized_piece.allowed_bin_ids);

    if (geom::validate_polygon(normalized_piece.polygon).is_valid() == false) {
      return std::unexpected(util::Status::invalid_input);
    }
    if (normalized_piece.geometry_revision == 0U) {
      normalized_piece.geometry_revision =
          geom::polygon_revision(normalized_piece.polygon);
    }
    normalized.request.pieces.push_back(std::move(normalized_piece));
  }

  if (normalized.request.pieces.empty()) {
    return std::unexpected(util::Status::invalid_input);
  }

  std::unordered_set<std::uint32_t> used_bin_ids;
  for (const auto &bin : normalized.request.bins) {
    for (std::uint32_t stock_index = 0; stock_index < bin.stock;
         ++stock_index) {
      normalized.expanded_bins.push_back({
          .source_bin_id = bin.bin_id,
          .expanded_bin_id =
              detail::derive_expanded_id(bin.bin_id, stock_index, used_bin_ids),
          .stock_index = stock_index,
          .identity =
              {
                  .lifecycle = pack::BinLifecycle::user_created,
                  .source_request_bin_id = bin.bin_id,
              },
      });
    }
  }

  std::unordered_set<std::uint32_t> used_piece_ids;
  for (const auto &piece : normalized.request.pieces) {
    for (std::uint32_t instance_index = 0; instance_index < piece.quantity;
         ++instance_index) {
      normalized.expanded_pieces.push_back({
          .source_piece_id = piece.piece_id,
          .expanded_piece_id = detail::derive_expanded_id(
              piece.piece_id, instance_index, used_piece_ids),
          .instance_index = instance_index,
      });
      normalized.forced_rotations.push_back(std::nullopt);
    }
  }

  return normalized;
}

auto normalize_nesting_request(const NestingRequest &request)
    -> std::expected<NestingRequest, util::Status> {
  const auto normalized = normalize_request(request);
  if (!normalized.has_value()) {
    return std::unexpected(normalized.error());
  }
  return normalized.value().request;
}

auto to_nesting_request(const ProfileRequest &request)
    -> std::expected<NestingRequest, util::Status> {
  if (const auto v = request.validate(); !v) {
    return std::unexpected(v.error());
  }

  const auto preset = api::profile_preset(request.profile);
  if (!preset.production.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
  }

  NestingRequest translated{};
  translated.bins = request.bins;
  translated.preprocess = request.preprocess;
  translated.execution.strategy = preset.strategy;
  translated.execution.production_optimizer = preset.production_optimizer;
  translated.execution.objective_mode = request.objective_mode;
  translated.execution.allow_part_overflow = request.allow_part_overflow;
  translated.execution.maintain_bed_assignment = request.maintain_bed_assignment;
  translated.execution.selected_bin_ids = request.selected_bin_ids;
  translated.execution.production = preset.production;

  translated.pieces.reserve(request.pieces.size());
  for (const auto &piece : request.pieces) {
    auto translated_piece = piece;
    if (request.maintain_bed_assignment) {
      if (piece.assigned_bin_id.has_value()) {
        translated_piece.allowed_bin_ids = {*piece.assigned_bin_id};
      } else if (translated_piece.allowed_bin_ids.empty()) {
        return std::unexpected(util::Status::invalid_input);
      }
    }
    translated_piece.allowed_bin_ids =
        detail::sort_unique_ids(std::move(translated_piece.allowed_bin_ids));
    translated.pieces.push_back(std::move(translated_piece));
  }

  if (!translated.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
  }

  return translated;
}

auto to_bounding_box_decoder_request(const NormalizedRequest &request)
    -> std::expected<pack::DecoderRequest, util::Status> {
  if (!request.request.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
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
      return std::unexpected(util::Status::invalid_input);
    }

    decoder_request.bins.push_back({
        .bin_id = expanded_bin.expanded_bin_id,
        .polygon = source_it->polygon,
        .geometry_revision = source_it->geometry_revision,
        .start_corner = source_it->start_corner,
        .source_request_bin_id = expanded_bin.source_bin_id,
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
        std::find_if(request.request.pieces.begin(),
                     request.request.pieces.end(), [&](const auto &piece) {
                       return piece.piece_id == expanded_piece.source_piece_id;
                     });
    if (source_it == request.request.pieces.end()) {
      return std::unexpected(util::Status::invalid_input);
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
                                     bins_it->second.begin(),
                                     bins_it->second.end());
      }
      piece.allowed_bin_ids = detail::sort_unique_ids(piece.allowed_bin_ids);
    }

    decoder_request.pieces.push_back(std::move(piece));
  }

  decoder_request.allow_part_overflow =
      request.request.execution.allow_part_overflow;

  if (!decoder_request.config.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
  }
  return decoder_request;
}

} // namespace shiny::nesting
