#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "internal/request_normalization.hpp"

namespace shiny::nesting::io {

enum class ImportedPathSegmentKind : std::uint8_t {
  line = 0,
  cubic_bezier = 1,
};

struct ImportedPathSegment {
  ImportedPathSegmentKind kind{ImportedPathSegmentKind::line};
  geom::Point2 start{};
  geom::Point2 control1{};
  geom::Point2 control2{};
  geom::Point2 end{};
};

struct ImportedRing {
  std::vector<ImportedPathSegment> segments{};
  bool closed{true};
};

struct ImportedShape {
  ImportedRing outer{};
  std::vector<ImportedRing> holes{};
};

struct ImportedPiece {
  std::uint32_t piece_id{0};
  std::uint32_t quantity{1};
  ImportedShape shape{};
  std::optional<geom::DiscreteRotationSet> allowed_rotations{};
  place::PartGrainCompatibility grain_compatibility{
      place::PartGrainCompatibility::unrestricted};
  std::vector<std::uint32_t> allowed_bin_ids{};
};

struct ImportedBin {
  std::uint32_t bin_id{0};
  std::uint32_t stock{1};
  ImportedShape shape{};
  place::PlacementStartCorner start_corner{
      place::PlacementStartCorner::bottom_left};
  std::vector<place::BedExclusionZone> exclusion_zones{};
};

struct ImportPreprocessOptions {
  double flatten_tolerance{0.25};
  double simplify_epsilon{0.0};
  bool normalize_piece_origins{true};
  bool discard_empty_bins{true};
};

struct ImportPreprocessRequest {
  std::vector<ImportedBin> bins{};
  std::vector<ImportedPiece> pieces{};
  NestingRequest base_request{};
  ImportPreprocessOptions options{};
};

[[nodiscard]] auto flatten_ring(const ImportedRing &ring, double tolerance)
    -> util::StatusOr<geom::Ring>;

[[nodiscard]] auto
preprocess_import_request(const ImportPreprocessRequest &request)
    -> util::StatusOr<NormalizedRequest>;

} // namespace shiny::nesting::io
