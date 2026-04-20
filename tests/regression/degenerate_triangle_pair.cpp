#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "nfp/convex_nfp.hpp"
#include "predicates/point_location.hpp"

namespace {

auto signed_area(const shiny::nesting::geom::Ring &ring) -> long double {
  long double area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    area += static_cast<long double>(ring[index].x) * ring[next_index].y -
            static_cast<long double>(ring[next_index].x) * ring[index].y;
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

TEST_CASE("regression nearly degenerate triangles keep a normalized loop",
          "[regression][nfp]") {
  const auto result = shiny::nesting::compute_convex_nfp({
      .piece_a_id = 30,
      .piece_b_id = 31,
      .convex_a = {{0.0, 0.0}, {5.0, 0.0}, {0.001, 0.0005}},
      .convex_b = {{0.0, 0.0}, {3.0, 0.0}, {0.0008, 0.0004}},
  });

  const auto &outer = first_outer_loop(result);
  REQUIRE(std::count_if(
              result.loops.begin(), result.loops.end(), [](const auto &loop) {
                return loop.kind == shiny::nesting::NfpFeatureKind::outer_loop;
              }) == 1);
  REQUIRE(shiny::nesting::pred::locate_point_in_ring({0.0, 0.0}, outer.vertices)
              .location != shiny::nesting::pred::PointLocation::exterior);
  REQUIRE(signed_area(outer.vertices) > 0.0L);
  REQUIRE(signed_area(outer.vertices) < 100.0L);
}