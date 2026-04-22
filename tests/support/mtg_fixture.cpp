#include "support/mtg_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

// We are the only translation unit in this binary that needs nanosvg, so
// we own the implementation here.
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

#include "geometry/types.hpp"
#include "observer.hpp"
#include "packing/config.hpp"
#include "packing/layout.hpp"
#include "placement/config.hpp"
#include "placement/types.hpp"
#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::test::mtg {

namespace {

constexpr double kInvariantTolerance = 1e-3;

[[nodiscard]] auto resolve_fixture_root() -> std::filesystem::path {
  if (const char *env =
          std::getenv("SHINY_NESTING_ENGINE_TEST_FIXTURE_ROOT")) {
    if (env[0] != '\0') {
      return std::filesystem::path{env};
    }
  }
  // Fallback: walk upwards from cwd until we find the fixture file.
  auto candidate = std::filesystem::current_path();
  for (int hop = 0; hop < 8; ++hop) {
    auto try_path = candidate / "tests" / "fixtures";
    if (std::filesystem::exists(try_path / "integration" / "mtg_test.svg")) {
      return try_path;
    }
    if (!candidate.has_parent_path() || candidate == candidate.parent_path()) {
      break;
    }
    candidate = candidate.parent_path();
  }
  throw std::runtime_error{
      "mtg_fixture: cannot resolve fixture root (set "
      "SHINY_NESTING_ENGINE_TEST_FIXTURE_ROOT)"};
}

[[nodiscard]] auto rect_polygon(double min_x, double min_y, double max_x,
                                double max_y) -> geom::PolygonWithHoles {
  geom::PolygonWithHoles polygon{};
  polygon.outer = {
      {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
  return polygon;
}

[[nodiscard]] auto polygon_aabb(const geom::PolygonWithHoles &polygon)
    -> geom::Box2 {
  geom::Box2 box{{0.0, 0.0}, {0.0, 0.0}};
  if (polygon.outer.empty()) {
    return box;
  }
  box.min = polygon.outer.front();
  box.max = polygon.outer.front();
  for (const auto &p : polygon.outer) {
    box.min.x = std::min(box.min.x, p.x);
    box.min.y = std::min(box.min.y, p.y);
    box.max.x = std::max(box.max.x, p.x);
    box.max.y = std::max(box.max.y, p.y);
  }
  return box;
}

struct ParsedShape {
  std::string id;
  double min_x{};
  double min_y{};
  double max_x{};
  double max_y{};
};

[[nodiscard]] auto parse_svg_shapes(const std::filesystem::path &path)
    -> std::vector<ParsedShape> {
  const auto path_string = path.string();
  std::unique_ptr<NSVGimage, void (*)(NSVGimage *)> image{
      nsvgParseFromFile(path_string.c_str(), "px", 96.0F), nsvgDelete};
  if (!image) {
    throw std::runtime_error{"mtg_fixture: failed to parse SVG: " +
                             path.string()};
  }
  std::vector<ParsedShape> shapes;
  for (const auto *shape = image->shapes; shape != nullptr;
       shape = shape->next) {
    ParsedShape parsed{};
    parsed.id = shape->id;
    parsed.min_x = shape->bounds[0];
    parsed.min_y = shape->bounds[1];
    parsed.max_x = shape->bounds[2];
    parsed.max_y = shape->bounds[3];
    shapes.push_back(std::move(parsed));
  }
  return shapes;
}

// Maps "path-N-M" shape id back to its owning artwork id ("N"); returns
// std::nullopt for shapes that are not artwork paths.
[[nodiscard]] auto artwork_id_from_shape_id(std::string_view shape_id)
    -> std::optional<std::uint32_t> {
  constexpr std::string_view prefix = "path-";
  if (shape_id.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  auto rest = shape_id.substr(prefix.size());
  auto dash = rest.find('-');
  if (dash == std::string_view::npos || dash == 0) {
    return std::nullopt;
  }
  std::uint32_t id = 0;
  for (std::size_t i = 0; i < dash; ++i) {
    if (rest[i] < '0' || rest[i] > '9') {
      return std::nullopt;
    }
    id = id * 10U + static_cast<std::uint32_t>(rest[i] - '0');
  }
  return id;
}

[[nodiscard]] auto bed_id_from_shape_id(std::string_view shape_id)
    -> std::optional<std::uint32_t> {
  constexpr std::string_view prefix = "working-area-";
  if (shape_id.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  auto rest = shape_id.substr(prefix.size());
  std::uint32_t id = 0;
  for (char c : rest) {
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    id = id * 10U + static_cast<std::uint32_t>(c - '0');
  }
  return id;
}

[[nodiscard]] auto inset_polygon(const geom::PolygonWithHoles &polygon,
                                 const BedMargins &margins)
    -> geom::PolygonWithHoles {
  if (!margins.any()) {
    return polygon;
  }
  const auto box = polygon_aabb(polygon);
  const double min_x = box.min.x + margins.left;
  const double min_y = box.min.y + margins.bottom;
  const double max_x = box.max.x - margins.right;
  const double max_y = box.max.y - margins.top;
  if (max_x - min_x <= 0.0 || max_y - min_y <= 0.0) {
    throw std::runtime_error{"mtg_fixture: margins exceed bed extent"};
  }
  return rect_polygon(min_x, min_y, max_x, max_y);
}

[[nodiscard]] auto bed_polygon_for_request(const MtgFixture & /*fixture*/,
                                           const MtgBed &bed,
                                           const MtgRequestOptions &options)
    -> geom::PolygonWithHoles {
  const auto &margins =
      bed.bed_id == kBed1Id ? options.bed1_margins_mm : options.bed2_margins_mm;
  return inset_polygon(bed.polygon, margins);
}

[[nodiscard]] auto contains_box(const geom::Box2 &outer, const geom::Box2 &inner)
    -> bool {
  return inner.min.x >= outer.min.x - kInvariantTolerance &&
         inner.min.y >= outer.min.y - kInvariantTolerance &&
         inner.max.x <= outer.max.x + kInvariantTolerance &&
         inner.max.y <= outer.max.y + kInvariantTolerance;
}

[[nodiscard]] auto boxes_overlap(const geom::Box2 &a, const geom::Box2 &b,
                                 double spacing) -> bool {
  // Treat boxes as overlapping if they intersect when each is inflated
  // by spacing/2 on every side. Equivalent to checking gap < spacing.
  const double half = spacing * 0.5;
  return !(a.max.x + half <= b.min.x - half ||
           b.max.x + half <= a.min.x - half ||
           a.max.y + half <= b.min.y - half ||
           b.max.y + half <= a.min.y - half);
}

[[nodiscard]] auto piece_lookup(const MtgFixture &fixture)
    -> std::unordered_map<std::uint32_t, const MtgPiece *> {
  std::unordered_map<std::uint32_t, const MtgPiece *> map;
  map.reserve(fixture.pieces.size());
  for (const auto &piece : fixture.pieces) {
    map.emplace(piece.piece_id, &piece);
  }
  return map;
}

}  // namespace

auto load_mtg_fixture() -> MtgFixture {
  const auto root = resolve_fixture_root();
  const auto svg_path = root / "integration" / "mtg_test.svg";

  MtgFixture fixture{};
  fixture.source_path = svg_path;

  const auto shapes = parse_svg_shapes(svg_path);

  // First pass: bed rectangles.
  std::unordered_map<std::uint32_t, ParsedShape> bed_shapes;
  for (const auto &shape : shapes) {
    if (auto bid = bed_id_from_shape_id(shape.id); bid.has_value()) {
      bed_shapes.emplace(*bid, shape);
    }
  }
  if (!bed_shapes.contains(kBed1Id) || !bed_shapes.contains(kBed2Id)) {
    throw std::runtime_error{
        "mtg_fixture: SVG missing working-area-1 or working-area-2"};
  }

  auto build_bed = [](std::uint32_t bed_id, const ParsedShape &s) -> MtgBed {
    MtgBed bed{};
    bed.bed_id = bed_id;
    bed.width_mm = s.max_x - s.min_x;
    bed.height_mm = s.max_y - s.min_y;
    bed.polygon = rect_polygon(0.0, 0.0, bed.width_mm, bed.height_mm);
    return bed;
  };
  fixture.bed1 = build_bed(kBed1Id, bed_shapes.at(kBed1Id));
  fixture.bed2 = build_bed(kBed2Id, bed_shapes.at(kBed2Id));

  // Second pass: aggregate per-artwork AABB by walking every "path-N-M".
  struct ArtworkAccum {
    std::uint32_t artwork_id{0};
    double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};
    bool seeded{false};
  };
  std::unordered_map<std::uint32_t, ArtworkAccum> accums;

  for (const auto &shape : shapes) {
    auto art = artwork_id_from_shape_id(shape.id);
    if (!art.has_value()) {
      continue;
    }
    auto &a = accums[*art];
    if (!a.seeded) {
      a.artwork_id = *art;
      a.min_x = shape.min_x;
      a.min_y = shape.min_y;
      a.max_x = shape.max_x;
      a.max_y = shape.max_y;
      a.seeded = true;
    } else {
      a.min_x = std::min(a.min_x, shape.min_x);
      a.min_y = std::min(a.min_y, shape.min_y);
      a.max_x = std::max(a.max_x, shape.max_x);
      a.max_y = std::max(a.max_y, shape.max_y);
    }
  }

  // Sort by artwork id for deterministic piece ordering.
  std::vector<ArtworkAccum> ordered;
  ordered.reserve(accums.size());
  for (auto &kv : accums) {
    ordered.push_back(kv.second);
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const auto &a, const auto &b) {
              return a.artwork_id < b.artwork_id;
            });

  // Source-bed assignment: containment of the artwork AABB centroid.
  const auto bed1_box = polygon_aabb(rect_polygon(
      bed_shapes.at(kBed1Id).min_x, bed_shapes.at(kBed1Id).min_y,
      bed_shapes.at(kBed1Id).max_x, bed_shapes.at(kBed1Id).max_y));
  const auto bed2_box = polygon_aabb(rect_polygon(
      bed_shapes.at(kBed2Id).min_x, bed_shapes.at(kBed2Id).min_y,
      bed_shapes.at(kBed2Id).max_x, bed_shapes.at(kBed2Id).max_y));

  std::uint32_t next_piece_id = 1;
  for (const auto &a : ordered) {
    const double cx = 0.5 * (a.min_x + a.max_x);
    const double cy = 0.5 * (a.min_y + a.max_y);
    std::uint32_t source_bed = 0;
    if (cx >= bed1_box.min.x && cx <= bed1_box.max.x &&
        cy >= bed1_box.min.y && cy <= bed1_box.max.y) {
      source_bed = kBed1Id;
    } else if (cx >= bed2_box.min.x && cx <= bed2_box.max.x &&
               cy >= bed2_box.min.y && cy <= bed2_box.max.y) {
      source_bed = kBed2Id;
    } else {
      throw std::runtime_error{
          "mtg_fixture: artwork " + std::to_string(a.artwork_id) +
          " centroid lies outside both beds"};
    }

    MtgPiece piece{};
    piece.piece_id = next_piece_id++;
    piece.label = "artwork-" + std::to_string(a.artwork_id);
    piece.source_bed_id = source_bed;
    piece.width_mm = a.max_x - a.min_x;
    piece.height_mm = a.max_y - a.min_y;
    piece.polygon = rect_polygon(0.0, 0.0, piece.width_mm, piece.height_mm);
    fixture.pieces.push_back(std::move(piece));
  }

  if (fixture.pieces.size() != kBaselinePieceCount) {
    throw std::runtime_error{
        "mtg_fixture: expected " + std::to_string(kBaselinePieceCount) +
        " pieces but parsed " + std::to_string(fixture.pieces.size())};
  }

  return fixture;
}

auto make_request(const MtgFixture &fixture, const MtgRequestOptions &options)
    -> NestingRequest {
  NestingRequest request{};

  // Bins.
  auto build_bin = [&](const MtgBed &bed,
                       const std::optional<place::PlacementStartCorner> &corner,
                       const std::vector<place::BedExclusionZone> &exclusions)
      -> BinRequest {
    BinRequest bin{};
    bin.bin_id = bed.bed_id;
    bin.polygon = bed_polygon_for_request(fixture, bed, options);
    bin.stock = 1;
    bin.geometry_revision = bed.bed_id;
    if (corner.has_value()) {
      bin.start_corner = *corner;
    }
    bin.exclusion_zones = exclusions;
    for (auto &zone : bin.exclusion_zones) {
      if (!zone.bin_id.has_value()) {
        zone.bin_id = bed.bed_id;
      }
    }
    return bin;
  };
  request.bins.push_back(
      build_bin(fixture.bed1, options.bed1_start_corner, options.bed1_exclusions));
  request.bins.push_back(
      build_bin(fixture.bed2, options.bed2_start_corner, options.bed2_exclusions));

  // Pieces.
  const bool pinned =
      options.maintain_bed_assignment || !options.allow_part_overflow;
  for (const auto &piece : fixture.pieces) {
    PieceRequest req{};
    req.piece_id = piece.piece_id;
    req.polygon = piece.polygon;
    req.quantity = 1;
    req.geometry_revision = piece.piece_id;
    req.priority = 0;
    req.allow_mirror = false;
    if (pinned) {
      req.allowed_bin_ids = {piece.source_bed_id};
    }
    request.pieces.push_back(std::move(req));
  }

  // Execution policy.
  request.execution.strategy = options.strategy;
  request.execution.production_optimizer = options.production_optimizer;
  request.execution.placement_policy = options.placement_policy;
  request.execution.part_spacing = options.part_spacing_mm;
  request.execution.selected_bin_ids = options.selected_bin_ids;
  request.execution.bounding_box = options.bounding_box;
  request.execution.deterministic_attempts.max_attempts =
      options.bounding_box_deterministic_attempts;
  request.execution.irregular = options.irregular;
  request.execution.production = options.production;
  request.execution.simulated_annealing = options.simulated_annealing;
  request.execution.alns = options.alns;
  request.execution.gdrr = options.gdrr;
  request.execution.lahc = options.lahc;

  return request;
}

auto make_rect_exclusion(std::uint32_t zone_id, std::uint32_t bin_id,
                         double min_x, double min_y, double max_x, double max_y)
    -> place::BedExclusionZone {
  place::BedExclusionZone zone{};
  zone.zone_id = zone_id;
  zone.bin_id = bin_id;
  zone.region.outer = {
      {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
  return zone;
}

auto bed1_half_block_exclusion() -> ExclusionRect {
  // Bed1 is 1133.86 x 1889.76mm. Block the top-left quadrant
  // (in bed-local coords with Y increasing upward) to displace ~half
  // of the bed1 area without occluding the entire bin.
  return ExclusionRect{
      .min_x = 0.0, .min_y = 945.0, .max_x = 567.0, .max_y = 1889.76};
}

void validate_layout(const MtgFixture &fixture, const NestingRequest &request,
                     const MtgRequestOptions &options,
                     const NestingResult &result,
                     const ExpectedOutcome &expected) {
  const auto pieces = piece_lookup(fixture);
  const double spacing = options.part_spacing_mm;
  const double effective_spacing = std::max(0.0, spacing - kInvariantTolerance);

  // Build bin-box lookup from the actual request bins so unknown bin ids
  // surface as a hard failure rather than silently aliasing to bed2.
  std::unordered_map<std::uint32_t, geom::Box2> bin_box_lookup;
  bin_box_lookup.reserve(request.bins.size());
  for (const auto &bin : request.bins) {
    bin_box_lookup.emplace(bin.bin_id, polygon_aabb(bin.polygon));
  }

  // Honor selected_bin_ids: every bin in result.layout.bins must be in
  // the requested selection (or all bins are allowed when selection is empty).
  if (!options.selected_bin_ids.empty()) {
    for (const auto &bin : result.layout.bins) {
      const bool allowed =
          std::find(options.selected_bin_ids.begin(),
                    options.selected_bin_ids.end(),
                    bin.bin_id) != options.selected_bin_ids.end();
      INFO("Bin " << bin.bin_id << " unexpectedly in result");
      REQUIRE(allowed);
    }
  }

  std::size_t total_placed = 0;
  std::unordered_map<std::uint32_t, std::size_t> per_bed_counts;
  std::unordered_map<std::uint32_t, std::uint32_t> piece_to_bin;
  std::unordered_map<std::uint32_t, std::size_t> placed_counts;

  for (const auto &bin : result.layout.bins) {
    per_bed_counts[bin.bin_id] = bin.placements.size();
    total_placed += bin.placements.size();

    auto box_it = bin_box_lookup.find(bin.bin_id);
    INFO("Result bin " << bin.bin_id << " not found in request bins");
    REQUIRE(box_it != bin_box_lookup.end());
    const auto &bin_box = box_it->second;
    REQUIRE(bin_box.max.x > bin_box.min.x);
    REQUIRE(bin_box.max.y > bin_box.min.y);

    // No-overlap + inside-bin checks within this bin.
    std::vector<geom::Box2> placement_boxes;
    placement_boxes.reserve(bin.placements.size());

    for (const auto &placement : bin.placements) {
      const auto piece_box = polygon_aabb(placement.polygon);
      placement_boxes.push_back(piece_box);

      INFO("Piece " << placement.placement.piece_id << " bin "
                    << bin.bin_id << " AABB ["
                    << piece_box.min.x << "," << piece_box.min.y << "]-["
                    << piece_box.max.x << "," << piece_box.max.y << "]");
      REQUIRE(contains_box(bin_box, piece_box));

      INFO("Placement bin_id " << placement.placement.bin_id
                               << " mismatches containing bin "
                               << bin.bin_id << " for piece "
                               << placement.placement.piece_id);
      REQUIRE(placement.placement.bin_id == bin.bin_id);

      // Allowed-bin restriction (engine-level model of maintain/overflow).
      auto it = pieces.find(placement.placement.piece_id);
      REQUIRE(it != pieces.end());
      const bool pinned = options.maintain_bed_assignment ||
                          !options.allow_part_overflow;
      if (pinned) {
        REQUIRE(bin.bin_id == it->second->source_bed_id);
      }
      piece_to_bin[placement.placement.piece_id] = bin.bin_id;
      ++placed_counts[placement.placement.piece_id];
    }

    for (std::size_t i = 0; i < placement_boxes.size(); ++i) {
      for (std::size_t j = i + 1; j < placement_boxes.size(); ++j) {
        INFO("Overlap check bin " << bin.bin_id << " pieces "
                                  << bin.placements[i].placement.piece_id
                                  << " vs "
                                  << bin.placements[j].placement.piece_id);
        REQUIRE_FALSE(boxes_overlap(placement_boxes[i], placement_boxes[j],
                                    effective_spacing));
      }
    }

    // Exclusion zones are checked exact (no spacing slack) — engine contract is hard disjointness.
    for (const auto &zone : expected.exclusions_to_check) {
      if (zone.bin_id.has_value() && *zone.bin_id != bin.bin_id) {
        continue;
      }
      const auto zone_box = polygon_aabb(
          geom::PolygonWithHoles{.outer = zone.region.outer});
      for (std::size_t i = 0; i < placement_boxes.size(); ++i) {
        INFO("Piece " << bin.placements[i].placement.piece_id
                      << " overlaps exclusion zone " << zone.zone_id);
        REQUIRE_FALSE(boxes_overlap(placement_boxes[i], zone_box, 0.0));
      }
    }
  }

  // Conservation + per-piece presence/uniqueness invariants. Sum quantities
  // from the request so a future quantity>1 fixture still validates.
  std::size_t expected_total_unique_with_quantity = 0;
  for (const auto &p : request.pieces) {
    expected_total_unique_with_quantity += p.quantity;
  }
  std::unordered_map<std::uint32_t, std::size_t> unplaced_counts;
  for (auto pid : result.layout.unplaced_piece_ids) {
    ++unplaced_counts[pid];
  }
  INFO("Conservation: placed " << total_placed << " + unplaced "
                                << result.layout.unplaced_piece_ids.size()
                                << " != expected total "
                                << expected_total_unique_with_quantity);
  REQUIRE(total_placed + result.layout.unplaced_piece_ids.size() ==
          expected_total_unique_with_quantity);

  for (const auto &p : request.pieces) {
    const auto placed_n = placed_counts.count(p.piece_id) != 0U
                              ? placed_counts.at(p.piece_id)
                              : 0U;
    const auto unplaced_n = unplaced_counts.count(p.piece_id) != 0U
                                ? unplaced_counts.at(p.piece_id)
                                : 0U;
    INFO("Per-piece presence piece " << p.piece_id << " placed " << placed_n
                                      << " unplaced " << unplaced_n
                                      << " quantity " << p.quantity);
    REQUIRE(placed_n + unplaced_n == p.quantity);
  }

  if (expected.expected_placed_count.has_value()) {
    INFO("Expected " << *expected.expected_placed_count << " placed, got "
                     << total_placed << " (unplaced "
                     << result.layout.unplaced_piece_ids.size() << ")");
    REQUIRE(total_placed == *expected.expected_placed_count);
  }

  for (const auto &[bid, count] : expected.per_bed_counts) {
    INFO("Per-bed count check bin " << bid);
    REQUIRE(per_bed_counts[bid] == count);
  }

  for (const auto &[piece_id, bin_id] : expected.required_assignments) {
    INFO("Required assignment piece " << piece_id << " bin " << bin_id);
    auto it = piece_to_bin.find(piece_id);
    REQUIRE(it != piece_to_bin.end());
    REQUIRE(it->second == bin_id);
  }

  if (expected.expected_stop_reason.has_value()) {
    INFO("Stop reason got "
         << static_cast<int>(result.stop_reason) << " expected "
         << static_cast<int>(*expected.expected_stop_reason));
    REQUIRE(result.stop_reason == *expected.expected_stop_reason);
  }

  if (expected.iteration_cap.has_value()) {
    INFO("Budget iterations " << result.budget.iterations_completed
                              << " vs cap " << *expected.iteration_cap);
    REQUIRE(result.budget.iterations_completed <= *expected.iteration_cap);
  }

  if (expected.time_cap_ms.has_value()) {
    // Allow generous slack since the engine checks time at iteration
    // boundaries; this is a sanity ceiling, not a strict budget.
    const std::uint64_t slack = std::max<std::uint64_t>(
        500U, *expected.time_cap_ms);
    INFO("Budget elapsed_ms " << result.budget.elapsed_milliseconds
                              << " vs cap " << *expected.time_cap_ms
                              << " + slack " << slack);
    REQUIRE(result.budget.elapsed_milliseconds <=
            *expected.time_cap_ms + slack);
  }
}

auto hash_bin_placements(const NestingResult &result, std::uint32_t bin_id)
    -> std::uint64_t {
  std::uint64_t hash = 1469598103934665603ULL;  // FNV offset basis
  auto mix = [&](std::uint64_t v) {
    hash ^= v;
    hash *= 1099511628211ULL;
  };
  auto mix_double = [&](double d) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &d, sizeof(double));
    mix(bits);
  };
  for (const auto &bin : result.layout.bins) {
    if (bin.bin_id != bin_id) {
      continue;
    }
    mix(bin.placements.size());
    for (const auto &p : bin.placements) {
      mix(p.placement.piece_id);
      mix(p.placement.rotation_index.value);
      mix_double(p.placement.translation.x);
      mix_double(p.placement.translation.y);
    }
  }
  return hash;
}

}  // namespace shiny::nesting::test::mtg
