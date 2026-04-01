#include <iostream>

#include "nfp/convex_ifp.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/nonconvex_nfp.hpp"

namespace {

using shiny::nfp::ConvexIfpRequest;
using shiny::nfp::ConvexNfpRequest;
using shiny::nfp::NonconvexNfpRequest;
using shiny::nfp::cache::AlgorithmRevision;
using shiny::nfp::geom::PolygonWithHoles;
using shiny::nfp::geom::Ring;

auto make_rectangle(double width, double height) -> Ring {
  return {{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}};
}

auto make_l_shape() -> PolygonWithHoles {
  return {
      .outer = {{0.0, 0.0},
                {4.0, 0.0},
                {4.0, 1.0},
                {1.0, 1.0},
                {1.0, 4.0},
                {0.0, 4.0}},
  };
}

void print_result(std::string_view label, const shiny::nfp::NfpResult &result) {
  std::size_t loop_count = 0;
  for (const auto &loop : result.loops) {
    if (loop.kind == shiny::nfp::NfpFeatureKind::outer_loop ||
        loop.kind == shiny::nfp::NfpFeatureKind::hole) {
      ++loop_count;
    }
  }

  std::cout << label
            << ": algorithm=" << shiny::nfp::to_string(result.algorithm)
            << ", loops=" << loop_count
            << ", perfect-fit-points=" << result.perfect_fit_points.size()
            << ", sliding-segments=" << result.perfect_sliding_segments.size()
            << '\n';
}

} // namespace

auto main() -> int {
  const auto convex_nfp = shiny::nfp::compute_convex_nfp(ConvexNfpRequest{
      .piece_a_id = 1,
      .piece_b_id = 2,
      .convex_a = make_rectangle(4.0, 2.0),
      .convex_b = make_rectangle(2.0, 2.0),
  });

  const auto convex_ifp = shiny::nfp::compute_convex_ifp(ConvexIfpRequest{
      .container_id = 10,
      .piece_id = 11,
      .container = {.outer = make_rectangle(8.0, 6.0)},
      .piece = {.outer = make_rectangle(2.0, 2.0)},
  });

  const auto nonconvex_nfp =
      shiny::nfp::compute_nonconvex_graph_nfp(NonconvexNfpRequest{
          .piece_a_id = 20,
          .piece_b_id = 21,
          .piece_a = make_l_shape(),
          .piece_b = make_l_shape(),
          .algorithm_revision = AlgorithmRevision{1},
      });

  print_result("Convex NFP", convex_nfp);
  print_result("Convex IFP", convex_ifp);
  print_result("Nonconvex NFP", nonconvex_nfp.result);
  return 0;
}