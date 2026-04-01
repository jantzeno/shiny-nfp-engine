#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

#include "packing/decoder.hpp"
#include "search/jostle.hpp"

namespace {

using shiny::nfp::geom::PolygonWithHoles;
using shiny::nfp::pack::BinPrototype;
using shiny::nfp::pack::PackingConfig;
using shiny::nfp::pack::PieceInput;
using shiny::nfp::place::PlacementPolicy;
using shiny::nfp::search::JostleSearch;
using shiny::nfp::search::SearchRequest;
using shiny::nfp::search::SearchResult;

auto make_rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return {
      .outer =
          {
              {.x = min_x, .y = min_y},
              {.x = max_x, .y = min_y},
              {.x = max_x, .y = max_y},
              {.x = min_x, .y = max_y},
          },
  };
}

auto make_piece(std::uint32_t piece_id, double width, double height,
                std::uint64_t geometry_revision,
                shiny::nfp::place::PartGrainCompatibility grain_compatibility =
                    shiny::nfp::place::PartGrainCompatibility::unrestricted)
    -> PieceInput {
  return {
      .piece_id = piece_id,
      .polygon = make_rectangle(0.0, 0.0, width, height),
      .geometry_revision = geometry_revision,
      .grain_compatibility = grain_compatibility,
  };
}

auto to_string(shiny::nfp::place::BedGrainDirection direction)
    -> std::string_view {
  switch (direction) {
  case shiny::nfp::place::BedGrainDirection::unrestricted:
    return "unrestricted";
  case shiny::nfp::place::BedGrainDirection::along_x:
    return "along_x";
  case shiny::nfp::place::BedGrainDirection::along_y:
    return "along_y";
  }
  return "unknown";
}

auto count_grain_constrained_pieces(const SearchRequest &request)
    -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      request.decoder_request.pieces.begin(),
      request.decoder_request.pieces.end(), [](const PieceInput &piece) {
        return piece.grain_compatibility !=
               shiny::nfp::place::PartGrainCompatibility::unrestricted;
      }));
}

auto make_request() -> SearchRequest {
  SearchRequest request{};
  request.decoder_request = {
      .bin =
          BinPrototype{
              .base_bin_id = 5,
              .polygon = make_rectangle(0.0, 0.0, 6.0, 4.0),
              .geometry_revision = 100,
          },
      .pieces =
          {
              make_piece(
                  1, 4.0, 2.0, 1,
                  shiny::nfp::place::PartGrainCompatibility::parallel_to_bed),
              make_piece(2, 2.0, 2.0, 2),
              make_piece(3, 2.0, 4.0, 3),
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          PackingConfig{
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                      .bed_grain_direction =
                          shiny::nfp::place::BedGrainDirection::along_x,
                      .exclusion_zones = {{
                          .zone_id = 17,
                          .region = {.outer = {{5.0, 3.0},
                                               {6.0, 3.0},
                                               {6.0, 4.0},
                                               {5.0, 4.0}}},
                      }},
                  },
              .enable_hole_first_placement = false,
          },
      .max_bin_count = 1,
  };

  request.local_search = {
      .max_iterations = 8,
      .deterministic_seed = 7,
  };

  return request;
}

void print_piece_order(std::string_view label,
                       const std::vector<std::uint32_t> &piece_order) {
  std::cout << label << ": ";
  for (std::size_t index = 0; index < piece_order.size(); ++index) {
    if (index > 0U) {
      std::cout << ", ";
    }
    std::cout << piece_order[index];
  }
  std::cout << '\n';
}

void print_summary(const SearchRequest &request, const SearchResult &result) {
  std::cout << "algorithm=" << shiny::nfp::to_string(result.algorithm) << '\n';
  std::cout << "Bed grain direction: "
            << to_string(
                   request.decoder_request.config.placement.bed_grain_direction)
            << '\n';
  std::cout << "Constrained pieces: " << count_grain_constrained_pieces(request)
            << '\n';
  std::cout << "Exclusion zones: "
            << request.decoder_request.config.placement.exclusion_zones.size()
            << '\n';
  std::cout << "Deterministic seed: " << result.deterministic_seed << '\n';
  std::cout << "Iterations completed: " << result.iterations_completed << '\n';
  std::cout << "Layouts evaluated: " << result.evaluated_layout_count << '\n';
  std::cout << "Reevaluation cache hits: " << result.reevaluation_cache_hits
            << '\n';
  std::cout << "Improved: " << (result.improved() ? "yes" : "no") << '\n';
  std::cout << "Baseline unplaced pieces: "
            << result.baseline.unplaced_piece_count << '\n';
  std::cout << "Best unplaced pieces: " << result.best.unplaced_piece_count
            << '\n';
  std::cout << "Best bin count: " << result.best.bin_count << '\n';
  std::cout << "Best total utilization: " << result.best.total_utilization
            << '\n';
  print_piece_order("Baseline piece order", result.baseline.piece_order);
  print_piece_order("Best piece order", result.best.piece_order);
}

} // namespace

auto main() -> int {
  JostleSearch search;
  const auto request = make_request();
  const auto result = search.improve(request);
  print_summary(request, result);
  return 0;
}