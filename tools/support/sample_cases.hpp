#pragma once

#include <cstdint>

#include "decomposition/decompose.hpp"
#include "nfp/requests.hpp"
#include "packing/decoder.hpp"
#include "packing/masonry.hpp"
#include "search/masonry.hpp"
#include "search/request.hpp"

namespace shiny::nfp::tooling {

inline auto make_rectangle(double min_x, double min_y, double max_x,
                           double max_y) -> geom::PolygonWithHoles {
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

inline auto make_l_shape_polygon() -> geom::PolygonWithHoles {
  return {
      .outer =
          {
              {.x = 0.0, .y = 0.0},
              {.x = 4.0, .y = 0.0},
              {.x = 4.0, .y = 1.0},
              {.x = 1.0, .y = 1.0},
              {.x = 1.0, .y = 4.0},
              {.x = 0.0, .y = 4.0},
          },
  };
}

inline auto make_piece(std::uint32_t piece_id, double width, double height,
                       std::uint64_t geometry_revision,
                       place::PartGrainCompatibility grain_compatibility =
                           place::PartGrainCompatibility::unrestricted)
    -> pack::PieceInput {
  return {
      .piece_id = piece_id,
      .polygon = make_rectangle(0.0, 0.0, width, height),
      .geometry_revision = geometry_revision,
      .grain_compatibility = grain_compatibility,
  };
}

inline auto make_bins(pack::BinInput base_bin, const std::size_t count)
    -> std::vector<pack::BinInput> {
  std::vector<pack::BinInput> bins;
  bins.reserve(std::max<std::size_t>(count, 1));
  for (std::size_t index = 0; index < std::max<std::size_t>(count, 1); ++index) {
    pack::BinInput bin = base_bin;
    bin.bin_id = base_bin.bin_id + static_cast<std::uint32_t>(index);
    bins.push_back(std::move(bin));
  }
  return bins;
}

inline auto make_decoder_request() -> pack::DecoderRequest {
  return {
      .bins = make_bins({.bin_id = 5,
                         .polygon = make_rectangle(0.0, 0.0, 6.0, 4.0),
                         .geometry_revision = 100},
                        1),
      .pieces =
          {
              make_piece(1, 4.0, 2.0, 1,
                         place::PartGrainCompatibility::parallel_to_bed),
              make_piece(2, 2.0, 2.0, 2),
              make_piece(3, 2.0, 4.0, 3),
          },
      .policy = place::PlacementPolicy::bottom_left,
      .config =
          pack::PackingConfig{
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                      .bed_grain_direction = place::BedGrainDirection::along_x,
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
  };
}

inline auto make_search_request() -> search::SearchRequest {
  return {
      .decoder_request = make_decoder_request(),
      .local_search =
          {
              .max_iterations = 8,
              .deterministic_seed = 7,
          },
  };
}

inline auto make_dense_search_request() -> search::SearchRequest {
  search::SearchRequest request{};
  request.decoder_request = {
      .bins = make_bins({.bin_id = 21,
                         .polygon = make_rectangle(0.0, 0.0, 16.0, 10.0),
                         .geometry_revision = 210},
                        2),
      .pieces =
          {
              make_piece(31, 5.0, 3.0, 31),
              make_piece(32, 4.0, 4.0, 32),
              make_piece(33, 3.0, 5.0, 33),
              make_piece(34, 6.0, 2.0, 34),
              make_piece(35, 2.0, 6.0, 35),
              make_piece(36, 4.0, 3.0, 36),
              make_piece(37, 3.0, 3.0, 37),
              make_piece(38, 2.0, 5.0, 38),
          },
      .policy = place::PlacementPolicy::bottom_left,
      .config =
          pack::PackingConfig{
              .placement =
                  {
                      .part_clearance = 0.05,
                      .allowed_rotations = {.angles_degrees = {0.0, 90.0}},
                      .bed_grain_direction =
                          place::BedGrainDirection::unrestricted,
                      .exclusion_zones = {{
                          .zone_id = 23,
                          .region = {.outer = {{13.5, 7.5},
                                               {16.0, 7.5},
                                               {16.0, 10.0},
                                               {13.5, 10.0}}},
                      }},
                  },
              .enable_hole_first_placement = false,
          },
  };
  request.local_search = {
      .max_iterations = 10,
      .deterministic_seed = 19,
      .plateau_budget = 4,
  };
  request.execution.control.capture_timestamps = false;
  return request;
}

inline auto make_genetic_search_request() -> search::SearchRequest {
  auto request = make_search_request();
  request.local_search = {
      .max_iterations = 3,
      .deterministic_seed = 11,
      .plateau_budget = 2,
  };
  request.genetic_search = {
      .max_generations = 6,
      .population_size = 8,
      .deterministic_seed = 17,
      .mutation_rate_percent = 20,
      .elite_count = 2,
      .tournament_size = 3,
      .plateau_generations = 3,
      .enable_local_search_polish = true,
      .enabled = true,
  };
  request.execution.control.capture_timestamps = false;
  return request;
}

inline auto make_masonry_request() -> pack::MasonryRequest {
  return {
       .decoder_request =
           {
               .bins = make_bins({.bin_id = 9,
                                  .polygon = make_rectangle(0.0, 0.0, 10.0,
                                                            10.0),
                                  .geometry_revision = 900},
                                 1),
               .pieces =
                   {
                      make_piece(11, 6.0, 4.0, 11),
                      make_piece(12, 4.0, 4.0, 12),
                      make_piece(13, 5.0, 3.0, 13),
                      make_piece(14, 2.0, 2.0, 14),
                  },
              .policy = place::PlacementPolicy::bottom_left,
              .config =
                  pack::PackingConfig{
                      .placement =
                          {
                              .allowed_rotations = {.angles_degrees = {0.0,
                                                                       90.0}},
                          },
                       .enable_hole_first_placement = false,
                   },
           },
      .masonry =
          {
              .fill_existing_shelves_first = true,
              .shelf_alignment_tolerance = 1e-6,
          },
  };
}

inline auto make_masonry_run_request() -> search::MasonryRunRequest {
  search::MasonryRunRequest request{};
  request.masonry_request = make_masonry_request();
  request.execution.control.capture_timestamps = false;
  return request;
}

inline auto make_l_shape_decomposition_request()
    -> decomp::DecompositionRequest {
  return {
      .piece_id = 42,
      .polygon = make_l_shape_polygon(),
      .rotation = {.degrees = 0.0},
      .algorithm =
          decomp::DecompositionAlgorithm::cgal_optimal_convex_partition,
  };
}

inline auto make_rectangle_hexagon_convex_nfp_request() -> ConvexNfpRequest {
  return {
      .piece_a_id = 5,
      .piece_b_id = 6,
      .convex_a =
          {
              {.x = 0.0, .y = 0.0},
              {.x = 4.0, .y = 0.0},
              {.x = 4.0, .y = 2.0},
              {.x = 0.0, .y = 2.0},
          },
      .convex_b =
          {
              {.x = 1.0, .y = 0.0},
              {.x = 3.0, .y = 0.0},
              {.x = 4.0, .y = 1.0},
              {.x = 3.0, .y = 2.0},
              {.x = 1.0, .y = 2.0},
              {.x = 0.0, .y = 1.0},
          },
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
  };
}

inline auto make_nonconvex_l_shape_rectangle_request() -> NonconvexNfpRequest {
  return {
      .piece_a_id = 101,
      .piece_b_id = 102,
      .piece_a = make_l_shape_polygon(),
      .piece_b = make_rectangle(0.0, 0.0, 3.0, 2.0),
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
      .algorithm_revision = cache::AlgorithmRevision{1},
  };
}

} // namespace shiny::nfp::tooling
