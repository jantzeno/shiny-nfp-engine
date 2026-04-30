#include <cstdint>
#include <iostream>
#include <vector>

#include "packing/bounding_box_packer.hpp"

namespace {

using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::BinInput;
using shiny::nesting::pack::BoundingBoxHeuristic;
using shiny::nesting::pack::BoundingBoxPacker;
using shiny::nesting::pack::DecoderRequest;
using shiny::nesting::pack::PieceInput;

auto make_rectangle(double width, double height) -> PolygonWithHoles {
  return {
      .outer =
          {
              {.x = 0.0, .y = 0.0},
              {.x = width, .y = 0.0},
              {.x = width, .y = height},
              {.x = 0.0, .y = height},
          },
  };
}

} // namespace

auto main() -> int {
  DecoderRequest request{
      .bins = {{
          .bin_id = 1,
          .polygon = make_rectangle(10.0, 6.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              {.piece_id = 101,
               .polygon = make_rectangle(4.0, 2.0),
               .geometry_revision = 1},
              {.piece_id = 102,
               .polygon = make_rectangle(3.0, 2.0),
               .geometry_revision = 2},
              {.piece_id = 103,
               .polygon = make_rectangle(2.0, 2.0),
               .geometry_revision = 3},
          },
  };
  request.config.bounding_box.heuristic = BoundingBoxHeuristic::skyline;
  request.config.deterministic_attempts.max_attempts = 3;
  request.config.placement.allowed_rotations.angles_degrees = {0.0, 90.0};

  BoundingBoxPacker packer;
  const auto attempts = packer.decode_attempts(request);
  if (attempts.empty()) {
    std::cerr << "no packing attempts completed\n";
    return 1;
  }

  const auto &best = attempts.front();
  std::cout << "attempts=" << attempts.size() << '\n';
  std::cout << "placed=" << best.layout.placement_trace.size() << '\n';
  std::cout << "unplaced=" << best.layout.unplaced_piece_ids.size() << '\n';
  for (const auto &entry : best.layout.placement_trace) {
    std::cout << "piece=" << entry.piece_id << " bin=" << entry.bin_id
              << " translation=(" << entry.translation.x << ", "
              << entry.translation.y
              << ") rotation=" << entry.resolved_rotation.degrees << '\n';
  }

  return 0;
}
