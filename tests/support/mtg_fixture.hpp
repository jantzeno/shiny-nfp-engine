#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "geometry/types.hpp"
#include "observer.hpp"
#include "packing/config.hpp"
#include "placement/config.hpp"
#include "placement/types.hpp"
#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::test::mtg {

// Source bed identifiers chosen to match the SVG `working-area-N` ids.
inline constexpr std::uint32_t kBed1Id = 1;
inline constexpr std::uint32_t kBed2Id = 2;

inline constexpr std::uint32_t kBaselinePieceCount = 18;

struct MtgPiece {
  std::uint32_t piece_id{0};   // Sequential 1..N
  std::string label{};         // SVG data-im2d-label (e.g. "Part 14")
  std::uint32_t source_bed_id{0};
  geom::PolygonWithHoles polygon{};  // Local-coord rectangle (origin-normalized AABB)
  double width_mm{0.0};
  double height_mm{0.0};
};

struct MtgBed {
  std::uint32_t bed_id{0};
  double width_mm{0.0};
  double height_mm{0.0};
  // Polygon at origin (0,0)-(width,height).
  geom::PolygonWithHoles polygon{};
};

struct MtgFixture {
  std::filesystem::path source_path{};
  MtgBed bed1{};
  MtgBed bed2{};
  std::vector<MtgPiece> pieces{};
};

[[nodiscard]] auto load_mtg_fixture() -> MtgFixture;

// Per-side margins (mm) applied to a bed by shrinking its polygon. Mirrors
// `BuildUsablePartbedPolygon` in
// `export_face/src/nesting/nesting_adapter.cpp` (lines 406-420).
struct BedMargins {
  double left{0.0};
  double right{0.0};
  double top{0.0};
  double bottom{0.0};

  [[nodiscard]] auto any() const -> bool {
    return left > 0.0 || right > 0.0 || top > 0.0 || bottom > 0.0;
  }
};

// Per-bed overrides keyed by bed id (1 or 2). Unset entries fall back to
// defaults.
struct MtgRequestOptions {
  StrategyKind strategy{StrategyKind::bounding_box};
  ProductionOptimizerKind production_optimizer{
      ProductionOptimizerKind::brkga};

  pack::BoundingBoxPackingConfig bounding_box{};
  std::uint32_t bounding_box_deterministic_attempts{1};

  IrregularOptions irregular{};
  ProductionSearchConfig production{};
  SAConfig simulated_annealing{};
  ALNSConfig alns{};
  GDRRConfig gdrr{};
  LAHCConfig lahc{};

  place::PlacementPolicy placement_policy{place::PlacementPolicy::bottom_left};
  double part_spacing_mm{0.0};

  // Engine-level model of the export_face flag pair (mirrors
  // `nesting_adapter.cpp:1174-1178`):
  //   maintain=true,  overflow=*     -> pinned to source bed
  //   maintain=false, overflow=false -> pinned to source bed
  //   maintain=false, overflow=true  -> free to move between beds
  bool maintain_bed_assignment{false};
  bool allow_part_overflow{true};

  // Empty -> all beds.
  std::vector<std::uint32_t> selected_bin_ids{};

  std::optional<place::PlacementStartCorner> bed1_start_corner{};
  std::optional<place::PlacementStartCorner> bed2_start_corner{};

  BedMargins bed1_margins_mm{};
  BedMargins bed2_margins_mm{};

  std::vector<place::BedExclusionZone> bed1_exclusions{};
  std::vector<place::BedExclusionZone> bed2_exclusions{};
};

[[nodiscard]] auto make_request(const MtgFixture &fixture,
                                const MtgRequestOptions &options)
    -> NestingRequest;

// Helper: rectangular exclusion zone in bed-local coordinates.
[[nodiscard]] auto make_rect_exclusion(std::uint32_t zone_id,
                                       std::uint32_t bin_id, double min_x,
                                       double min_y, double max_x, double max_y)
    -> place::BedExclusionZone;

struct ExpectedOutcome {
  // Total placed pieces expected; std::nullopt skips the check.
  std::optional<std::size_t> expected_placed_count{};
  // Optional per-bed placed counts (bed_id -> count).
  std::vector<std::pair<std::uint32_t, std::size_t>> per_bed_counts{};
  // Pieces that must end up on a specific bed (piece_id -> bed_id).
  std::vector<std::pair<std::uint32_t, std::uint32_t>> required_assignments{};
  // Stop reason expected; std::nullopt skips the check.
  std::optional<StopReason> expected_stop_reason{};
  // If set, every placed piece must avoid this exclusion polygon (per bin).
  std::vector<place::BedExclusionZone> exclusions_to_check{};
  // Iteration cap to verify against `result.budget.iterations_completed`.
  std::optional<std::size_t> iteration_cap{};
  // Time cap (ms) to verify against `result.budget.elapsed_milliseconds`.
  std::optional<std::uint64_t> time_cap_ms{};
  // Tolerance for spacing / overlap checks (mm).
  double tolerance_mm{1e-4};
};

// Asserts the eight invariants in plan.md against the result. Uses Catch2
// REQUIRE macros, so it must be called from within a TEST_CASE / SECTION.
//
// AABB-validation assumption: overlap, containment, and exclusion-zone
// checks are AABB-based. They are exact only because every fixture piece
// is an axis-aligned rectangle; adding non-rectangular fixtures will
// require polygon-true checks.
void validate_layout(const MtgFixture &fixture, const NestingRequest &request,
                     const MtgRequestOptions &options,
                     const NestingResult &result,
                     const ExpectedOutcome &expected);

// Helper used by the start-corner sensitivity check: returns a deterministic
// hash of placements for a given bin id.
[[nodiscard]] auto hash_bin_placements(const NestingResult &result,
                                       std::uint32_t bin_id) -> std::uint64_t;

// A canonical "half-bed-1" exclusion used by Section J.
struct ExclusionRect {
  double min_x{};
  double min_y{};
  double max_x{};
  double max_y{};
};
[[nodiscard]] auto bed1_half_block_exclusion() -> ExclusionRect;

}  // namespace shiny::nesting::test::mtg
