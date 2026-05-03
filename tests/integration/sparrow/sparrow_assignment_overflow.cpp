// Integration tests verifying that the fill-first constructive engine output
// becomes a correct Sparrow warm-start seed while preserving the assignment
// and overflow contracts established in Milestone 3.
//
// These tests cover checkpoint 08c from the port plan: fill-first layout is
// exported to Sparrow seed state via export_sparrow_seed(), and the seed
// faithfully captures overflow lineage, bin assignment pinning, and unplaced
// piece tracking so Sparrow search can resume from a valid constructive start.

#include <algorithm>
#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "solve.hpp"
#include "packing/constructive/fill_first_engine.hpp"
#include "packing/sparrow/adapters/layout_adapter.hpp"
#include "packing/sparrow/solution.hpp"
#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

using namespace shiny::nesting;

namespace {

[[nodiscard]] auto rectangle(double min_x, double min_y, double max_x,
                             double max_y) -> geom::PolygonWithHoles {
  return geom::PolygonWithHoles(geom::Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

[[nodiscard]] auto find_lineage_for_template(
    const std::vector<pack::sparrow::PortOverflowLineage> &lineage,
    std::uint32_t template_bin_id)
    -> const pack::sparrow::PortOverflowLineage * {
  for (const auto &entry : lineage) {
    if (entry.template_bin_id == template_bin_id) {
      return &entry;
    }
  }
  return nullptr;
}

[[nodiscard]] auto find_placement_for_piece(
    const std::vector<pack::sparrow::PortPlacement> &placements,
    std::uint32_t piece_id) -> const pack::sparrow::PortPlacement * {
  for (const auto &pl : placements) {
    if (pl.piece_id == piece_id) {
      return &pl;
    }
  }
  return nullptr;
}

} // namespace

TEST_CASE(
    "fill-first warm start captures overflow lineage so Sparrow can "
    "recover the bin frontier",
    "[sparrow][constructive-seed][overflow][integration]") {
  // One 4×4 bin. Two 4×4 pieces. With allow_part_overflow=true the second
  // piece must go to an engine-created overflow bin. The Sparrow warm-start
  // seed must record that overflow lineage so search workers can respect the
  // bin boundary structure established by the constructive phase.
  constexpr std::uint32_t kTemplateBinId = 80;
  constexpr std::uint32_t kPiece1Id = 1;
  constexpr std::uint32_t kPiece2Id = 2;

  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.allow_part_overflow = true;
  request.execution.irregular.piece_ordering = PieceOrdering::input;
  request.bins.push_back(BinRequest{
      .bin_id = kTemplateBinId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = kPiece1Id,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = kPiece2Id,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });

  const auto result = solve(request, SolveControl{.random_seed = 0});
  REQUIRE(result.has_value());
  REQUIRE(result.value().validation.valid);

  // Both pieces must be placed — one on the template bin, one on overflow.
  REQUIRE(result.value().layout.unplaced_piece_ids.empty());

  // The constructive replay must record the overflow event.
  REQUIRE_FALSE(result.value().constructive.overflow_events.empty());

  // Export the Sparrow warm-start seed from the constructive result.
  const auto seed = pack::constructive::export_sparrow_seed(result.value());

  // All placed pieces appear in the seed's placement list.
  REQUIRE(seed.placements.size() == 2U);

  // The seed's multi-bin state captures the overflow lineage.
  REQUIRE_FALSE(seed.multi_bin.overflow_lineage.empty());

  const auto *lineage =
      find_lineage_for_template(seed.multi_bin.overflow_lineage, kTemplateBinId);
  REQUIRE(lineage != nullptr);
  REQUIRE(lineage->overflow_bin_id != kTemplateBinId);
  // source_request_bin_id traces back to the original request bin.
  REQUIRE(lineage->source_request_bin_id == kTemplateBinId);

  // The second piece must be on the overflow bin, not the template bin.
  const auto *p2 = find_placement_for_piece(seed.placements, kPiece2Id);
  REQUIRE(p2 != nullptr);
  REQUIRE(p2->bin_id == lineage->overflow_bin_id);

  // Seed round-trip: converting the seed back to a layout preserves all
  // placements and the overflow lineage.
  const auto round_trip =
      pack::sparrow::adapters::to_seed_solution(seed.layout, seed.constructive);
  REQUIRE(round_trip.placements.size() == seed.placements.size());
  REQUIRE(round_trip.multi_bin.overflow_lineage.size() ==
          seed.multi_bin.overflow_lineage.size());
}

TEST_CASE(
    "fill-first warm start preserves bin assignment pinning so Sparrow "
    "workers respect maintain_bed_assignment",
    "[sparrow][constructive-seed][maintain-bed-assignment][integration]") {
  // Two 4×4 bins. Two 4×4 pieces each carrying an assigned_bin_id that pins
  // them to distinct bins. With maintain_bed_assignment=true the constructive
  // engine must not move pieces across bins. The Sparrow seed must record the
  // pinned bin_id for each placement so search workers start from a valid
  // assignment.
  constexpr std::uint32_t kBinAId = 100;
  constexpr std::uint32_t kBinBId = 200;
  constexpr std::uint32_t kPieceAId = 10;
  constexpr std::uint32_t kPieceBId = 20;

  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.maintain_bed_assignment = true;
  request.allow_part_overflow = false;

  request.bins.push_back(BinRequest{
      .bin_id = kBinAId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = kBinBId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = kPieceAId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
      .assigned_bin_id = kBinAId,
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = kPieceBId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
      .assigned_bin_id = kBinBId,
  });

  const auto result = solve(request, ProfileSolveControl{.random_seed = 0});
  REQUIRE(result.has_value());
  REQUIRE(result.value().validation.valid);

  // Both pieces must be placed on their pinned bins.
  REQUIRE(result.value().layout.unplaced_piece_ids.empty());

  // No overflow bins should be created under maintain_bed_assignment.
  REQUIRE(result.value().constructive.overflow_events.empty());

  // Export the Sparrow warm-start seed.
  const auto seed = pack::constructive::export_sparrow_seed(result.value());
  REQUIRE(seed.placements.size() == 2U);

  // Each piece must appear on its assigned bin in the seed.
  const auto *pa = find_placement_for_piece(seed.placements, kPieceAId);
  REQUIRE(pa != nullptr);
  REQUIRE(pa->bin_id == kBinAId);

  const auto *pb = find_placement_for_piece(seed.placements, kPieceBId);
  REQUIRE(pb != nullptr);
  REQUIRE(pb->bin_id == kBinBId);

  // No overflow lineage: pinned mode never creates overflow bins.
  REQUIRE(seed.multi_bin.overflow_lineage.empty());
}

TEST_CASE(
    "fill-first warm start tracks unplaced pieces when assignment "
    "constraints prevent full placement",
    "[sparrow][constructive-seed][unplaced][integration]") {
  // Two bins but one piece has allowed_bin_ids that conflicts with
  // maintain_bed_assignment: piece is assigned to bin A but allowed_bin_ids
  // only lists bin B. The constructive engine cannot place it. The Sparrow seed
  // must record it in unplaced_piece_ids so search workers start from the
  // correct partial layout.
  constexpr std::uint32_t kBinAId = 300;
  constexpr std::uint32_t kBinBId = 301;
  constexpr std::uint32_t kPiecePlacedId = 30;
  constexpr std::uint32_t kPieceBlockedId = 31;

  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.maintain_bed_assignment = true;
  request.allow_part_overflow = false;

  request.bins.push_back(BinRequest{
      .bin_id = kBinAId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = kBinBId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  // Piece that can be placed: assigned to bin A, allowed only on A.
  request.pieces.push_back(PieceRequest{
      .piece_id = kPiecePlacedId,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
      .allowed_bin_ids = {kBinAId},
      .assigned_bin_id = kBinAId,
  });
  // Piece that cannot be placed: assigned to bin A (pinned), but allowed only
  // on B — the intersection is empty so the engine cannot place it.
  request.pieces.push_back(PieceRequest{
      .piece_id = kPieceBlockedId,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
      .allowed_bin_ids = {kBinBId},
      .assigned_bin_id = kBinAId,
  });

  const auto result = solve(request, ProfileSolveControl{.random_seed = 0});
  REQUIRE(result.has_value());

  // The blocked piece must appear in unplaced_piece_ids.
  const auto &unplaced = result.value().layout.unplaced_piece_ids;
  const bool blocked_unplaced =
      std::find(unplaced.begin(), unplaced.end(), kPieceBlockedId) !=
      unplaced.end();
  REQUIRE(blocked_unplaced);

  // The Sparrow seed must propagate the unplaced list.
  const auto seed = pack::constructive::export_sparrow_seed(result.value());
  const auto &seed_unplaced = seed.multi_bin.unplaced_piece_ids;
  const bool seed_blocked_unplaced =
      std::find(seed_unplaced.begin(), seed_unplaced.end(), kPieceBlockedId) !=
      seed_unplaced.end();
  REQUIRE(seed_blocked_unplaced);
}
