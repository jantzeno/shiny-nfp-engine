#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "geometry/polygon.hpp"
#include "internal/legacy_strategy.hpp"
#include "placement/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

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
  std::int32_t priority{0};
  double value{1.0};
  bool allow_mirror{false};
  // When provided, this narrows `ExecutionPolicy::default_rotations`.
  std::optional<geom::DiscreteRotationSet> allowed_rotations{};
  place::PartGrainCompatibility grain_compatibility{
      place::PartGrainCompatibility::unrestricted};
  // Effective bin eligibility is the intersection of these ids with
  // `ExecutionPolicy::selected_bin_ids`.
  std::vector<std::uint32_t> allowed_bin_ids{};
  std::optional<std::uint32_t> assigned_bin_id{};
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

enum class SolveProfile : std::uint8_t {
  quick = 0,
  balanced = 1,
  maximum_search = 2,
};

struct ProfileRequest {
  std::vector<BinRequest> bins{};
  std::vector<PieceRequest> pieces{};
  PreprocessPolicy preprocess{};
  SolveProfile profile{SolveProfile::balanced};
  ObjectiveMode objective_mode{ObjectiveMode::placement_count};
  std::optional<std::uint64_t> time_limit_milliseconds{};
  std::vector<std::uint32_t> selected_bin_ids{};
  bool allow_part_overflow{true};
  bool maintain_bed_assignment{false};

  /// Returns true if the request passes all basic validity checks:
  /// profile is recognized and any profile that requires a time limit has a
  /// non-zero one set.
  [[nodiscard]] auto is_valid() const -> bool;

  /// Returns an error status if the request fails basic validity checks,
  /// otherwise returns void. Prefer this over is_valid() when the caller
  /// needs to propagate the failure through a util::Expected<T> chain.
  [[nodiscard]] auto validate() const -> util::Expected<void>;
};

struct NestingRequest {
  std::vector<BinRequest> bins{};
  std::vector<PieceRequest> pieces{};
  PreprocessPolicy preprocess{};
  ExecutionPolicy execution{};

  [[nodiscard]] auto is_valid() const -> bool;
};

[[nodiscard]] auto to_nesting_request(const ProfileRequest &request)
    -> util::StatusOr<NestingRequest>;

[[nodiscard]] auto normalize_nesting_request(const NestingRequest &request)
    -> util::StatusOr<NestingRequest>;

} // namespace shiny::nesting
