#include <catch2/catch_test_macros.hpp>

#include "io/import_preprocess.hpp"

namespace {

using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::io::ImportedBin;
using shiny::nesting::io::ImportedPathSegment;
using shiny::nesting::io::ImportedPathSegmentKind;
using shiny::nesting::io::ImportedPiece;
using shiny::nesting::io::ImportedRing;
using shiny::nesting::io::ImportedShape;

auto line(const double x1, const double y1, const double x2, const double y2)
    -> ImportedPathSegment {
  return {
      .kind = ImportedPathSegmentKind::line,
      .start = shiny::nesting::geom::Point2(x1, y1),
      .end = shiny::nesting::geom::Point2(x2, y2),
  };
}

auto rectangle_ring(const double min_x, const double min_y, const double max_x,
                    const double max_y) -> ImportedRing {
  return {
      .segments =
          {
              line(min_x, min_y, max_x, min_y),
              line(max_x, min_y, max_x, max_y),
              line(max_x, max_y, min_x, max_y),
              line(min_x, max_y, min_x, min_y),
          },
      .closed = true,
  };
}

} // namespace

TEST_CASE("import preprocessing flattens bezier rings", "[io][preprocess]") {
  const ImportedRing ring{
      .segments =
          {
              {.kind = ImportedPathSegmentKind::cubic_bezier,
               .start = shiny::nesting::geom::Point2(0.0, 0.0),
               .control1 = shiny::nesting::geom::Point2(1.0, 2.0),
               .control2 = shiny::nesting::geom::Point2(2.0, 2.0),
               .end = shiny::nesting::geom::Point2(3.0, 0.0)},
              line(3.0, 0.0, 3.0, -1.0),
              line(3.0, -1.0, 0.0, -1.0),
              line(0.0, -1.0, 0.0, 0.0),
          },
      .closed = true,
  };

  const auto flattened = shiny::nesting::io::flatten_ring(ring, 0.1);
  REQUIRE(flattened.ok());
  REQUIRE(flattened.value().size() > 4);
}

TEST_CASE("import preprocessing owns bin filtering and piece normalization",
          "[io][preprocess]") {
  shiny::nesting::io::ImportPreprocessRequest request;
  request.base_request.execution.selected_bin_ids = {2};
  request.options.flatten_tolerance = 0.1;
  request.options.normalize_piece_origins = true;
  request.options.discard_empty_bins = true;
  request.bins = {
      {.bin_id = 1,
       .stock = 0,
       .shape = ImportedShape{.outer = rectangle_ring(0, 0, 10, 10)}},
      {.bin_id = 2,
       .stock = 1,
       .shape = ImportedShape{.outer = rectangle_ring(0, 0, 20, 20)}},
  };
  request.pieces = {{
      .piece_id = 9,
      .quantity = 1,
      .shape = ImportedShape{.outer = rectangle_ring(10, 15, 13, 19)},
      .allowed_bin_ids = {2},
  }};

  const auto normalized =
      shiny::nesting::io::preprocess_import_request(request);
  REQUIRE(normalized.ok());
  REQUIRE(normalized.value().request.bins.size() == 1);
  REQUIRE(normalized.value().request.bins.front().bin_id == 2);
  REQUIRE(normalized.value().request.pieces.size() == 1);

  const auto bounds = shiny::nesting::geom::compute_bounds(
      normalized.value().request.pieces.front().polygon);
  REQUIRE(bounds.min.x() == 0.0);
  REQUIRE(bounds.min.y() == 0.0);
  REQUIRE(bounds.max.x() == 3.0);
  REQUIRE(bounds.max.y() == 4.0);
}
