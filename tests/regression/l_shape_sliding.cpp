#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "nfp/nonconvex_nfp.hpp"
#include "predicates/point_location.hpp"

namespace {

auto signed_area(const shiny::nesting::geom::Ring &ring) -> long double {
  long double area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    area += static_cast<long double>(ring[index].x()) * ring[next_index].y() -
            static_cast<long double>(ring[next_index].x()) * ring[index].y();
  }
  return area / 2.0L;
}

auto first_outer_loop(const shiny::nesting::NfpResult &result)
    -> const shiny::nesting::NfpLoop & {
  const auto it = std::find_if(
      result.loops.begin(), result.loops.end(), [](const auto &loop) {
        return loop.kind == shiny::nesting::NfpFeatureKind::outer_loop;
      });
  REQUIRE(it != result.loops.end());
  return *it;
}

} // namespace

TEST_CASE("regression l-shape sliding preserves a boundary segment",
          "[regression][nfp]") {
  const auto result = shiny::nesting::compute_nonconvex_graph_nfp({
      .piece_a_id = 1,
      .piece_b_id = 2,
      .piece_a = shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}),
      .piece_b = shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 3.0},
                            {0.0, 3.0}}),
      .algorithm_revision = shiny::nesting::cache::AlgorithmRevision{1},
  });

  REQUIRE(result.status == shiny::nesting::ExtractionStatus::success);
  const auto &outer = first_outer_loop(result.result);
  REQUIRE(std::count_if(result.result.loops.begin(), result.result.loops.end(),
                        [](const auto &loop) {
                          return loop.kind ==
                                 shiny::nesting::NfpFeatureKind::outer_loop;
                        }) >= 1);
  REQUIRE(shiny::nesting::pred::locate_point_in_ring(outer.vertices.front(),
                                                     outer.vertices)
              .location != shiny::nesting::pred::PointLocation::exterior);
  REQUIRE(signed_area(outer.vertices) > 1.0L);
  REQUIRE(signed_area(outer.vertices) < 200.0L);
}