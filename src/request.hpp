#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "geometry/polygon.hpp"
#include "packing/config.hpp"
#include "packing/decoder.hpp"
#include "placement/config.hpp"
#include "placement/types.hpp"
#include "polygon_ops/simplify.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

enum class StrategyKind : std::uint8_t {
  bounding_box = 0,
  irregular_constructive = 1,
  irregular_production = 2,
};

struct PreprocessPolicy {
  double simplify_epsilon{0.0};
  bool normalize_piece_origins{true};
  bool discard_empty_bins{true};
};

struct PieceRequest {
  std::uint32_t piece_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint32_t quantity{1};
  std::uint64_t geometry_revision{0};
  std::optional<geom::DiscreteRotationSet> allowed_rotations{};
  place::PartGrainCompatibility grain_compatibility{
      place::PartGrainCompatibility::unrestricted};
  std::vector<std::uint32_t> allowed_bin_ids{};
};

struct BinRequest {
  std::uint32_t bin_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint32_t stock{1};
  std::uint64_t geometry_revision{0};
  place::PlacementStartCorner start_corner{
      place::PlacementStartCorner::bottom_left};
  std::vector<place::BedExclusionZone> exclusion_zones{};
};

struct ProductionSearchConfig {
  std::size_t population_size{24};
  std::size_t elite_count{6};
  std::size_t mutant_count{4};
  std::size_t max_generations{24};
  double elite_bias{0.7};
  std::size_t diversification_swaps{2};
  std::size_t polishing_passes{1};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct ExecutionPolicy {
  StrategyKind strategy{StrategyKind::bounding_box};
  place::PlacementPolicy placement_policy{place::PlacementPolicy::bottom_left};
  geom::DiscreteRotationSet default_rotations{{0.0, 90.0, 180.0, 270.0}};
  double part_spacing{0.0};
  bool enable_part_in_part_placement{false};
  bool explore_concave_candidates{false};
  std::vector<std::uint32_t> selected_bin_ids{};
  pack::BoundingBoxPackingConfig bounding_box{};
  pack::DeterministicAttemptConfig deterministic_attempts{};
  ProductionSearchConfig production{};
};

struct NestingRequest {
  std::vector<BinRequest> bins{};
  std::vector<PieceRequest> pieces{};
  PreprocessPolicy preprocess{};
  ExecutionPolicy execution{};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct ExpandedPieceInstance {
  std::uint32_t source_piece_id{0};
  std::uint32_t expanded_piece_id{0};
  std::uint32_t instance_index{0};
};

struct ExpandedBinInstance {
  std::uint32_t source_bin_id{0};
  std::uint32_t expanded_bin_id{0};
  std::uint32_t stock_index{0};
};

struct NormalizedRequest {
  NestingRequest request{};
  std::vector<ExpandedPieceInstance> expanded_pieces{};
  std::vector<ExpandedBinInstance> expanded_bins{};
};

[[nodiscard]] auto normalize_request(const NestingRequest &request)
    -> util::StatusOr<NormalizedRequest>;

[[nodiscard]] auto
to_bounding_box_decoder_request(const NormalizedRequest &request)
    -> util::StatusOr<pack::DecoderRequest>;

} // namespace shiny::nesting
