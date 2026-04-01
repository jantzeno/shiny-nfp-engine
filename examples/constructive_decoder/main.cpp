#include <iostream>

#include "algorithm_kind.hpp"
#include "packing/decoder.hpp"

namespace {

using shiny::nfp::geom::PolygonWithHoles;
using shiny::nfp::pack::BinPrototype;
using shiny::nfp::pack::ConstructiveDecoder;
using shiny::nfp::pack::DecoderRequest;
using shiny::nfp::pack::PieceInput;
using shiny::nfp::place::PlacementPolicy;

auto make_rectangle(double width, double height) -> PolygonWithHoles {
  return {
      .outer = {{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}},
  };
}

void print_layout(const shiny::nfp::pack::DecoderResult &result) {
  std::cout << "algorithm="
            << shiny::nfp::to_string(
                   shiny::nfp::AlgorithmKind::constructive_decoder)
            << '\n';
  std::cout << "Bins used: " << result.bins.size() << '\n';
  std::cout << "Unplaced pieces: " << result.layout.unplaced_piece_ids.size()
            << '\n';
  std::cout << "Placement trace:" << '\n';

  for (const auto &entry : result.layout.placement_trace) {
    std::cout << "  piece " << entry.piece_id << " -> bin " << entry.bin_id
              << " at (" << entry.translation.x << ", " << entry.translation.y
              << ") rotation-index " << entry.rotation_index.value;
    if (entry.opened_new_bin) {
      std::cout << " [opened new bin]";
    }
    std::cout << '\n';
  }
}

} // namespace

auto main() -> int {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bin =
          BinPrototype{
              .base_bin_id = 1,
              .polygon = make_rectangle(10.0, 10.0),
              .geometry_revision = 100,
          },
      .pieces =
          {
              PieceInput{.piece_id = 1,
                         .polygon = make_rectangle(5.0, 2.0),
                         .geometry_revision = 1},
              PieceInput{.piece_id = 2,
                         .polygon = make_rectangle(4.0, 3.0),
                         .geometry_revision = 2},
              PieceInput{.piece_id = 3,
                         .polygon = make_rectangle(3.0, 3.0),
                         .geometry_revision = 3},
              PieceInput{.piece_id = 4,
                         .polygon = make_rectangle(2.0, 6.0),
                         .geometry_revision = 4},
              PieceInput{.piece_id = 5,
                         .polygon = make_rectangle(1.5, 4.0),
                         .geometry_revision = 5},
          },
      .policy = PlacementPolicy::bottom_left,
  };

  const auto result = decoder.decode(request);
  print_layout(result);
  return 0;
}