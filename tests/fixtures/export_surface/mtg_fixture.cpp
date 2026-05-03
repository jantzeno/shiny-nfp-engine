#include "fixtures/export_surface/mtg_fixture.hpp"

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

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/operations/simplify.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/queries/sanitize.hpp"
#include "geometry/queries/validity.hpp"
#include "geometry/transforms/transform.hpp"
#include "geometry/types.hpp"
#include "internal/request_normalization.hpp"
#include "observer.hpp"
#include "packing/common.hpp"
#include "packing/config.hpp"
#include "packing/layout.hpp"
#include "placement/config.hpp"
#include "placement/types.hpp"
#include "request.hpp"
#include "result.hpp"

namespace shiny::nesting::test::mtg {

namespace {

constexpr double kInvariantTolerance = 1e-3;

[[nodiscard]] constexpr auto
validity_issue_name(const geom::PolygonValidityIssue issue) -> const char * {
  switch (issue) {
  case geom::PolygonValidityIssue::ok:
    return "ok";
  case geom::PolygonValidityIssue::non_finite_coordinate:
    return "non_finite_coordinate";
  case geom::PolygonValidityIssue::too_few_vertices:
    return "too_few_vertices";
  case geom::PolygonValidityIssue::zero_area:
    return "zero_area";
  case geom::PolygonValidityIssue::self_intersection:
    return "self_intersection";
  case geom::PolygonValidityIssue::hole_outside_outer:
    return "hole_outside_outer";
  case geom::PolygonValidityIssue::hole_intersection:
    return "hole_intersection";
  }
  return "unknown";
}

[[nodiscard]] auto resolve_fixture_root() -> std::filesystem::path {
  if (const char *env = std::getenv("SHINY_NESTING_ENGINE_TEST_FIXTURE_ROOT")) {
    if (env[0] != '\0') {
      return std::filesystem::path{env};
    }
  }
  // Fallback: walk upwards from cwd until we find the fixture file.
  auto candidate = std::filesystem::current_path();
  for (int hop = 0; hop < 8; ++hop) {
    auto try_path = candidate / "tests" / "fixtures";
    if (std::filesystem::exists(try_path / "export_surface" / "mtg_test.svg")) {
      return try_path;
    }
    if (!candidate.has_parent_path() || candidate == candidate.parent_path()) {
      break;
    }
    candidate = candidate.parent_path();
  }
  throw std::runtime_error{"mtg_fixture: cannot resolve fixture root (set "
                           "SHINY_NESTING_ENGINE_TEST_FIXTURE_ROOT)"};
}

[[nodiscard]] auto rect_polygon(double min_x, double min_y, double max_x,
                                double max_y) -> geom::PolygonWithHoles {
  geom::PolygonWithHoles polygon{};
  polygon.outer() = {
      {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
  return polygon;
}

[[nodiscard]] auto polygon_aabb(const geom::PolygonWithHoles &polygon)
    -> geom::Box2 {
  geom::Box2 box{{0.0, 0.0}, {0.0, 0.0}};
  if (polygon.outer().empty()) {
    return box;
  }
  box.min = polygon.outer().front();
  box.max = polygon.outer().front();
  for (const auto &p : polygon.outer()) {
    box.min.set_x(std::min(box.min.x(), p.x()));
    box.min.set_y(std::min(box.min.y(), p.y()));
    box.max.set_x(std::max(box.max.x(), p.x()));
    box.max.set_y(std::max(box.max.y(), p.y()));
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
  const double min_x = box.min.x() + margins.left;
  const double min_y = box.min.y() + margins.bottom;
  const double max_x = box.max.x() - margins.right;
  const double max_y = box.max.y() - margins.top;
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

[[nodiscard]] auto contains_box(const geom::Box2 &outer,
                                const geom::Box2 &inner) -> bool {
  return inner.min.x() >= outer.min.x() - kInvariantTolerance &&
         inner.min.y() >= outer.min.y() - kInvariantTolerance &&
         inner.max.x() <= outer.max.x() + kInvariantTolerance &&
         inner.max.y() <= outer.max.y() + kInvariantTolerance;
}

struct PieceValidationLookups {
  std::unordered_map<std::uint32_t, const MtgPiece *> fixture_piece_by_id;
  std::unordered_map<std::uint32_t, const PieceRequest *> request_piece_by_id;
};

[[nodiscard]] auto make_rect_bed(std::uint32_t bed_id, double width_mm,
                                 double height_mm) -> MtgBed {
  MtgBed bed{};
  bed.bed_id = bed_id;
  bed.width_mm = width_mm;
  bed.height_mm = height_mm;
  bed.polygon = rect_polygon(0.0, 0.0, width_mm, height_mm);
  return bed;
}

[[nodiscard]] auto make_rect_piece(std::uint32_t piece_id, std::string label,
                                   std::uint32_t source_bed_id, double width_mm,
                                   double height_mm) -> MtgPiece {
  MtgPiece piece{};
  piece.piece_id = piece_id;
  piece.label = std::move(label);
  piece.source_bed_id = source_bed_id;
  piece.width_mm = width_mm;
  piece.height_mm = height_mm;
  piece.polygon = rect_polygon(0.0, 0.0, width_mm, height_mm);
  return piece;
}

[[nodiscard]] auto effective_rotation_set(const PieceRequest &piece,
                                          const ExecutionPolicy &execution)
    -> const geom::DiscreteRotationSet & {
  return piece.allowed_rotations.has_value() ? *piece.allowed_rotations
                                             : execution.default_rotations;
}

[[nodiscard]] auto
resolve_piece_rotation(const PieceRequest &piece,
                       const ExecutionPolicy &execution,
                       const geom::RotationIndex rotation_index)
    -> std::optional<geom::ResolvedRotation> {
  return geom::resolve_rotation(rotation_index,
                                effective_rotation_set(piece, execution));
}

[[nodiscard]] auto build_piece_validation_lookups(const MtgFixture &fixture,
                                                  const NestingRequest &request)
    -> PieceValidationLookups {
  PieceValidationLookups lookups{};

  std::unordered_map<std::uint32_t, const MtgPiece *> source_fixture_pieces;
  source_fixture_pieces.reserve(fixture.pieces.size());
  for (const auto &piece : fixture.pieces) {
    source_fixture_pieces.emplace(piece.piece_id, &piece);
  }

  std::unordered_map<std::uint32_t, const PieceRequest *> source_request_pieces;
  source_request_pieces.reserve(request.pieces.size());
  for (const auto &piece : request.pieces) {
    source_request_pieces.emplace(piece.piece_id, &piece);
  }

  const auto normalized = normalize_request(request);
  if (normalized.has_value()) {
    for (const auto &expanded_piece : normalized.value().expanded_pieces) {
      const auto fixture_it =
          source_fixture_pieces.find(expanded_piece.source_piece_id);
      const auto request_it =
          source_request_pieces.find(expanded_piece.source_piece_id);
      if (fixture_it == source_fixture_pieces.end() ||
          request_it == source_request_pieces.end()) {
        continue;
      }
      lookups.fixture_piece_by_id.emplace(expanded_piece.expanded_piece_id,
                                          fixture_it->second);
      lookups.request_piece_by_id.emplace(expanded_piece.expanded_piece_id,
                                          request_it->second);
    }
    return lookups;
  }

  lookups.fixture_piece_by_id = std::move(source_fixture_pieces);
  lookups.request_piece_by_id = std::move(source_request_pieces);
  return lookups;
}

} // namespace

auto make_asymmetric_engine_surface_fixture() -> MtgFixture {
  MtgFixture fixture{};
  fixture.source_path =
      std::filesystem::path{"synthetic://asymmetric_engine_surface_fixture"};
  fixture.bed1 = make_rect_bed(kBed1Id, 16.0, 22.0);
  fixture.bed2 = make_rect_bed(kBed2Id, 22.0, 12.0);
  fixture.pieces.push_back(
      make_rect_piece(1, "rotates-into-bed1", kBed1Id, 20.0, 8.0));
  fixture.pieces.push_back(
      make_rect_piece(2, "bed2-square", kBed2Id, 4.0, 4.0));
  return fixture;
}

auto boxes_violate_spacing(const geom::Box2 &a, const geom::Box2 &b,
                           const double spacing_mm, const double tolerance_mm)
    -> bool {
  const double effective_spacing =
      std::max(0.0, spacing_mm - std::max(0.0, tolerance_mm));
  return pack::boxes_violate_spacing(a, b, effective_spacing);
}

auto load_mtg_fixture() -> MtgFixture {
  const auto root = resolve_fixture_root();
  const auto svg_path = root / "export_surface" / "mtg_test.svg";

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
  std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) {
    return a.artwork_id < b.artwork_id;
  });

  // Source-bed assignment: containment of the artwork AABB centroid.
  const auto bed1_box = polygon_aabb(
      rect_polygon(bed_shapes.at(kBed1Id).min_x, bed_shapes.at(kBed1Id).min_y,
                   bed_shapes.at(kBed1Id).max_x, bed_shapes.at(kBed1Id).max_y));
  const auto bed2_box = polygon_aabb(
      rect_polygon(bed_shapes.at(kBed2Id).min_x, bed_shapes.at(kBed2Id).min_y,
                   bed_shapes.at(kBed2Id).max_x, bed_shapes.at(kBed2Id).max_y));

  std::uint32_t next_piece_id = 1;
  for (const auto &a : ordered) {
    const double cx = 0.5 * (a.min_x + a.max_x);
    const double cy = 0.5 * (a.min_y + a.max_y);
    std::uint32_t source_bed = 0;
    if (cx >= bed1_box.min.x() && cx <= bed1_box.max.x() &&
        cy >= bed1_box.min.y() && cy <= bed1_box.max.y()) {
      source_bed = kBed1Id;
    } else if (cx >= bed2_box.min.x() && cx <= bed2_box.max.x() &&
               cy >= bed2_box.min.y() && cy <= bed2_box.max.y()) {
      source_bed = kBed2Id;
    } else {
      throw std::runtime_error{"mtg_fixture: artwork " +
                               std::to_string(a.artwork_id) +
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

namespace {

// Extracts polygon boundaries from a single nanosvg path. Each NSVGpath
// stores cubic-bezier control points as a flat array of length npts =
// 3*ncubic + 1, where every third point starting at index 0 is a curve
// endpoint (corner). For straight-line "M ... L ..." paths nanosvg still
// emits cubics with collinear control points, so taking the endpoints
// recovers the polygon vertices exactly.
[[nodiscard]] auto extract_path_polygon(const NSVGpath *path) -> geom::Polygon {
  geom::Polygon polygon{};
  if (path == nullptr || path->npts <= 0) {
    return polygon;
  }
  for (int i = 0; i + 1 <= path->npts; i += 3) {
    polygon.outer().push_back(
        geom::Point2{static_cast<double>(path->pts[2 * i]),
                     static_cast<double>(path->pts[2 * i + 1])});
  }
  if (polygon.outer().size() >= 2 && path->closed != 0) {
    const auto &front = polygon.outer().front();
    const auto &back = polygon.outer().back();
    if (std::abs(front.x() - back.x()) < 1e-9 &&
        std::abs(front.y() - back.y()) < 1e-9) {
      polygon.outer().pop_back();
    }
  }
  return polygon;
}

[[nodiscard]] auto
largest_by_area(const std::vector<geom::PolygonWithHoles> &polygons)
    -> geom::PolygonWithHoles {
  const geom::PolygonWithHoles *best = nullptr;
  double best_area = -1.0;
  for (const auto &p : polygons) {
    const double area = std::abs(geom::polygon_area(p));
    if (area > best_area) {
      best_area = area;
      best = &p;
    }
  }
  if (best == nullptr) {
    throw std::runtime_error{"mtg_fixture: empty union result"};
  }
  return *best;
}

[[nodiscard]] auto union_subpaths(const std::vector<geom::Polygon> &subpaths)
    -> geom::PolygonWithHoles {
  if (subpaths.empty()) {
    throw std::runtime_error{"mtg_fixture: artwork has no sub-paths"};
  }

  // Seed accumulator with the first sub-path that is geometrically valid.
  std::vector<geom::PolygonWithHoles> acc;
  std::size_t start = 0;
  for (; start < subpaths.size(); ++start) {
    if (subpaths[start].outer().size() >= 3) {
      geom::PolygonWithHoles seed{};
      seed.outer() = subpaths[start].outer();
      seed = geom::normalize_polygon(seed);
      seed = geom::simplify_polygon(seed);
      if (!seed.outer().empty()) {
        acc.push_back(std::move(seed));
        break;
      }
    }
  }
  if (acc.empty()) {
    throw std::runtime_error{"mtg_fixture: no valid sub-paths to union"};
  }

  for (std::size_t i = start + 1; i < subpaths.size(); ++i) {
    if (subpaths[i].outer().size() < 3) {
      continue;
    }
    geom::PolygonWithHoles next{};
    next.outer() = subpaths[i].outer();
    next = geom::normalize_polygon(next);
    next = geom::simplify_polygon(next);
    if (next.outer().empty()) {
      continue;
    }

    std::vector<geom::PolygonWithHoles> rebuilt;
    rebuilt.reserve(acc.size());
    geom::PolygonWithHoles current = std::move(next);
    for (const auto &existing : acc) {
      auto merged = geom::union_polygons(existing, current);
      if (merged.size() == 1) {
        current = std::move(merged.front());
      } else {
        // Disconnected from `current`: keep `existing` as-is.
        rebuilt.push_back(existing);
      }
    }
    rebuilt.push_back(std::move(current));
    acc = std::move(rebuilt);
  }

  // Take the largest connected component as the silhouette. Smaller
  // disconnected detail features (e.g., isolated decorative rectangles)
  // are dropped intentionally — they don't change the AABB and exposing
  // them as separate polygons would break the one-piece-per-artwork
  // contract used by the rest of the fixture.
  auto silhouette = largest_by_area(acc);
  // Drop holes: the test surface is the outer silhouette only; holes from
  // sub-path topology aren't meaningful for the bug-repro use case.
  silhouette.holes().clear();
  return silhouette;
}

[[nodiscard]] auto translate_polygon(const geom::PolygonWithHoles &polygon,
                                     double dx, double dy)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles out{};
  out.outer().reserve(polygon.outer().size());
  for (const auto &p : polygon.outer()) {
    out.outer().push_back(geom::Point2{p.x() + dx, p.y() + dy});
  }
  return out;
}

struct ParsedArtwork {
  std::uint32_t artwork_id{0};
  // Sub-paths in document coordinates.
  std::vector<geom::Polygon> subpaths;
  // AABB derived from the same nanosvg `shape->bounds` source the
  // rectangle fixture uses, for cross-validation.
  double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};
  bool seeded{false};
};

[[nodiscard]] auto parse_svg_artworks(const std::filesystem::path &path)
    -> std::unordered_map<std::uint32_t, ParsedArtwork> {
  const auto path_string = path.string();
  std::unique_ptr<NSVGimage, void (*)(NSVGimage *)> image{
      nsvgParseFromFile(path_string.c_str(), "px", 96.0F), nsvgDelete};
  if (!image) {
    throw std::runtime_error{"mtg_fixture: failed to parse SVG: " +
                             path.string()};
  }

  std::unordered_map<std::uint32_t, ParsedArtwork> artworks;
  for (const auto *shape = image->shapes; shape != nullptr;
       shape = shape->next) {
    auto art_id = artwork_id_from_shape_id(shape->id);
    if (!art_id.has_value()) {
      continue;
    }
    auto &accum = artworks[*art_id];
    accum.artwork_id = *art_id;
    if (!accum.seeded) {
      accum.min_x = shape->bounds[0];
      accum.min_y = shape->bounds[1];
      accum.max_x = shape->bounds[2];
      accum.max_y = shape->bounds[3];
      accum.seeded = true;
    } else {
      accum.min_x =
          std::min(accum.min_x, static_cast<double>(shape->bounds[0]));
      accum.min_y =
          std::min(accum.min_y, static_cast<double>(shape->bounds[1]));
      accum.max_x =
          std::max(accum.max_x, static_cast<double>(shape->bounds[2]));
      accum.max_y =
          std::max(accum.max_y, static_cast<double>(shape->bounds[3]));
    }
    for (const auto *p = shape->paths; p != nullptr; p = p->next) {
      auto poly = extract_path_polygon(p);
      if (poly.outer().size() >= 3) {
        accum.subpaths.push_back(std::move(poly));
      }
    }
  }
  return artworks;
}

} // namespace

auto load_mtg_fixture_with_actual_polygons() -> MtgFixture {
  // Start from the rectangle fixture so all bed/source/piece-id metadata
  // remains identical to the existing baseline.
  auto fixture = load_mtg_fixture();

  const auto artworks = parse_svg_artworks(fixture.source_path);

  // Build artwork-id -> piece lookup. Pieces are constructed in
  // ascending artwork-id order by `load_mtg_fixture`, but we re-resolve
  // the mapping by reparsing the AABB labels rather than relying on
  // index alignment.
  std::unordered_map<std::uint32_t, std::pair<double, double>>
      rect_aabb_by_artwork;
  for (const auto &kv : artworks) {
    rect_aabb_by_artwork.emplace(
        kv.first, std::make_pair(kv.second.max_x - kv.second.min_x,
                                 kv.second.max_y - kv.second.min_y));
  }

  // Build a piece index by deterministic artwork-id ordering, mirroring
  // the loop in `load_mtg_fixture`.
  std::vector<std::uint32_t> ordered_ids;
  ordered_ids.reserve(artworks.size());
  for (const auto &kv : artworks) {
    ordered_ids.push_back(kv.first);
  }
  std::sort(ordered_ids.begin(), ordered_ids.end());

  if (ordered_ids.size() != fixture.pieces.size()) {
    throw std::runtime_error{
        "mtg_fixture: artwork count drifted from rectangle fixture"};
  }

  for (std::size_t i = 0; i < ordered_ids.size(); ++i) {
    const auto art_id = ordered_ids[i];
    const auto &art = artworks.at(art_id);
    auto silhouette = union_subpaths(art.subpaths);

    // Origin-normalize: translate so that AABB.min == (0,0). The
    // rectangle fixture stores piece geometry in the same convention.
    const auto box = polygon_aabb(silhouette);
    silhouette = translate_polygon(silhouette, -box.min.x(), -box.min.y());
    const auto local_box = polygon_aabb(silhouette);

    auto &piece = fixture.pieces[i];

    // Cross-check that our extracted silhouette matches the AABB the
    // rectangle fixture computed from nanosvg's `shape->bounds`. A
    // mismatch larger than 1e-3 mm indicates a silently-dropped
    // sub-path or extraction bug.
    constexpr double kAabbTolerance = 1e-3;
    const double width = local_box.max.x() - local_box.min.x();
    const double height = local_box.max.y() - local_box.min.y();
    if (std::abs(width - piece.width_mm) > kAabbTolerance ||
        std::abs(height - piece.height_mm) > kAabbTolerance) {
      throw std::runtime_error{
          "mtg_fixture: extracted polygon AABB drifted from "
          "rectangle-fixture AABB for artwork " +
          std::to_string(art_id)};
    }

    if (silhouette.outer().size() < 3) {
      throw std::runtime_error{
          "mtg_fixture: extracted polygon has fewer than 3 vertices for "
          "artwork " +
          std::to_string(art_id)};
    }

    const auto sanitized = geom::sanitize_polygon(silhouette);
    const auto validity = geom::validate_polygon(sanitized.polygon);
    if (!validity.is_valid()) {
      throw std::runtime_error{
          "mtg_fixture: invalid extracted polygon for artwork " +
          std::to_string(art_id) +
          " issue=" + validity_issue_name(validity.issue)};
    }
    if (sanitized.sliver_rings > 0U) {
      throw std::runtime_error{
          "mtg_fixture: extracted polygon contains sliver rings for artwork " +
          std::to_string(art_id)};
    }

    piece.polygon = std::move(sanitized.polygon);
    piece.width_mm = width;
    piece.height_mm = height;
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
  request.bins.push_back(build_bin(fixture.bed1, options.bed1_start_corner,
                                   options.bed1_exclusions));
  request.bins.push_back(build_bin(fixture.bed2, options.bed2_start_corner,
                                   options.bed2_exclusions));

  // Pieces.
  const bool pinned = options.maintain_bed_assignment;
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
  request.execution.allow_part_overflow = options.allow_part_overflow;
  request.execution.selected_bin_ids = options.selected_bin_ids;
  request.execution.bounding_box = options.bounding_box;
  request.execution.deterministic_attempts.max_attempts =
      options.bounding_box_deterministic_attempts;
  request.execution.irregular = options.irregular;
  request.execution.production = options.production;

  return request;
}

auto make_rect_exclusion(std::uint32_t zone_id, std::uint32_t bin_id,
                         double min_x, double min_y, double max_x, double max_y)
    -> place::BedExclusionZone {
  place::BedExclusionZone zone{};
  zone.zone_id = zone_id;
  zone.bin_id = bin_id;
  zone.region.outer() = {
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
  const auto pieces = build_piece_validation_lookups(fixture, request);
  const double spacing_tolerance =
      std::max(expected.tolerance_mm, kInvariantTolerance);

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
    REQUIRE(bin_box.max.x() > bin_box.min.x());
    REQUIRE(bin_box.max.y() > bin_box.min.y());

    // No-overlap + inside-bin checks within this bin.
    std::vector<geom::Box2> placement_boxes;
    placement_boxes.reserve(bin.placements.size());

    for (const auto &placement : bin.placements) {
      const auto piece_box = polygon_aabb(placement.polygon);
      placement_boxes.push_back(piece_box);

      INFO("Piece " << placement.placement.piece_id << " bin " << bin.bin_id
                    << " AABB [" << piece_box.min.x() << ","
                    << piece_box.min.y() << "]-[" << piece_box.max.x() << ","
                    << piece_box.max.y() << "]");
      REQUIRE(contains_box(bin_box, piece_box));

      INFO("Placement bin_id "
           << placement.placement.bin_id << " mismatches containing bin "
           << bin.bin_id << " for piece " << placement.placement.piece_id);
      REQUIRE(placement.placement.bin_id == bin.bin_id);

      const auto fixture_piece_it =
          pieces.fixture_piece_by_id.find(placement.placement.piece_id);
      const auto request_piece_it =
          pieces.request_piece_by_id.find(placement.placement.piece_id);
      REQUIRE(fixture_piece_it != pieces.fixture_piece_by_id.end());
      REQUIRE(request_piece_it != pieces.request_piece_by_id.end());

      const auto *fixture_piece = fixture_piece_it->second;
      const auto *request_piece = request_piece_it->second;

      if (expected.require_rotation_admissibility) {
        const auto resolved_rotation =
            resolve_piece_rotation(*request_piece, request.execution,
                                   placement.placement.rotation_index);
        INFO("Piece " << placement.placement.piece_id << " rotation_index "
                      << placement.placement.rotation_index.value
                      << " must resolve within the admissible rotation set");
        REQUIRE(resolved_rotation.has_value());
      }

      if (expected.require_allowed_bin_admissibility &&
          !request_piece->allowed_bin_ids.empty()) {
        const bool bin_allowed =
            std::find(request_piece->allowed_bin_ids.begin(),
                      request_piece->allowed_bin_ids.end(),
                      placement.placement.bin_id) !=
            request_piece->allowed_bin_ids.end();
        INFO("Piece " << placement.placement.piece_id << " placed on bin "
                      << placement.placement.bin_id
                      << " outside its allowed_bin_ids contract");
        REQUIRE(bin_allowed);
      }

      // Allowed-bin restriction derived from the export_face pinning model.
      const bool pinned = options.maintain_bed_assignment;
      if (pinned) {
        REQUIRE(bin.bin_id == fixture_piece->source_bed_id);
      }
      piece_to_bin[placement.placement.piece_id] = bin.bin_id;
      ++placed_counts[placement.placement.piece_id];
    }

    for (std::size_t i = 0; i < placement_boxes.size(); ++i) {
      for (std::size_t j = i + 1; j < placement_boxes.size(); ++j) {
        INFO("Overlap check bin "
             << bin.bin_id << " pieces " << bin.placements[i].placement.piece_id
             << " vs " << bin.placements[j].placement.piece_id);
        const auto overlap_area =
            geom::polygon_area_sum(geom::intersection_polygons(
                bin.placements[i].polygon, bin.placements[j].polygon));
        REQUIRE(overlap_area <= spacing_tolerance);
        if (options.part_spacing_mm > 0.0) {
          const auto clearance = geom::polygon_distance(
              bin.placements[i].polygon, bin.placements[j].polygon);
          REQUIRE(clearance + spacing_tolerance >= options.part_spacing_mm);
        }
      }
    }

    // Exclusion zones are checked exact (no spacing slack) — engine contract is
    // hard disjointness.
    for (const auto &zone : expected.exclusions_to_check) {
      if (zone.bin_id.has_value() && *zone.bin_id != bin.bin_id) {
        continue;
      }
      const auto zone_box =
          polygon_aabb(geom::PolygonWithHoles(zone.region.outer()));
      for (std::size_t i = 0; i < placement_boxes.size(); ++i) {
        INFO("Piece " << bin.placements[i].placement.piece_id
                      << " overlaps exclusion zone " << zone.zone_id);
        REQUIRE_FALSE(
            boxes_violate_spacing(placement_boxes[i], zone_box, 0.0, 0.0));
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
    REQUIRE(result.placed_parts() == *expected.expected_placed_count);
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
}
auto hash_bin_placements(const NestingResult &result, std::uint32_t bin_id)
    -> std::uint64_t {
  std::uint64_t hash = 1469598103934665603ULL; // FNV offset basis
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
      mix_double(p.placement.translation.x());
      mix_double(p.placement.translation.y());
    }
  }
  return hash;
}

} // namespace shiny::nesting::test::mtg
