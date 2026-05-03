#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "api/dto.hpp"
#include "api/request_builder.hpp"
#include "runtime/progress.hpp"
#include "solve.hpp"
#include "support/fixture_test_support.hpp"

namespace {

namespace fs = std::filesystem;

using shiny::nesting::BinRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::ProfileProgressSnapshot;
using shiny::nesting::SolveProfile;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::api::ProfileRequestBuilder;
using shiny::nesting::api::ProfileSolveControlBuilder;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;

auto repo_root() -> fs::path {
  return shiny::nesting::test::fixture_root().parent_path().parent_path();
}

auto read_text(const fs::path &path) -> std::string {
  std::ifstream input(path);
  REQUIRE(input.is_open());
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto simple_profile_request(const SolveProfile profile,
                            const std::optional<std::uint64_t> time_limit_ms =
                                std::nullopt) -> ProfileRequestBuilder {
  auto builder = ProfileRequestBuilder{};
  builder.with_profile(profile)
      .add_bin(BinRequest{
          .bin_id = 11,
          .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
      })
      .add_bin(BinRequest{
          .bin_id = 22,
          .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
      })
      .add_piece(PieceRequest{
          .piece_id = 101,
          .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      });
  if (time_limit_ms.has_value()) {
    builder.with_time_limit_ms(*time_limit_ms);
  }
  return builder;
}

auto heavy_profile_request(const SolveProfile profile,
                           const std::uint64_t time_limit_ms)
    -> shiny::nesting::ProfileRequest {
  auto builder = ProfileRequestBuilder{};
  builder.with_profile(profile)
      .with_time_limit_ms(time_limit_ms)
      .add_bin(BinRequest{
          .bin_id = 70,
          .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
      });
  for (std::size_t index = 0; index < 18U; ++index) {
    builder.add_piece(PieceRequest{
        .piece_id = static_cast<std::uint32_t>(200U + index),
        .polygon = rectangle(0.0, 0.0, 4.0 + (index % 3U), 2.0 + (index % 2U)),
    });
  }
  const auto request = builder.build_checked();
  REQUIRE(request.ok());
  return request.value();
}

} // namespace

TEST_CASE("profile request builder exposes quick balanced and maximum search profiles",
          "[api][profiles]") {
  const auto request = simple_profile_request(SolveProfile::quick)
                           .with_selected_bins({11})
                           .with_allow_part_overflow(false)
                           .build_checked();

  REQUIRE(request.ok());

  const auto translated = shiny::nesting::to_nesting_request(request.value());
  REQUIRE(translated.ok());
  REQUIRE(translated.value().execution.strategy ==
          StrategyKind::bounding_box);
  REQUIRE(translated.value().execution.selected_bin_ids ==
          std::vector<std::uint32_t>{11U});
  REQUIRE_FALSE(translated.value().execution.allow_part_overflow);
}

TEST_CASE("profile request builder requires a time limit for balanced and maximum search",
          "[api][profiles]") {
  const auto missing_time_limit =
      simple_profile_request(SolveProfile::balanced).build_checked();
  REQUIRE_FALSE(missing_time_limit.ok());

  const auto balanced =
      simple_profile_request(SolveProfile::balanced, 1'000U).build_checked();
  const auto maximum =
      simple_profile_request(SolveProfile::maximum_search, 1'000U)
          .build_checked();

  REQUIRE(balanced.ok());
  REQUIRE(maximum.ok());

  const auto balanced_request =
      shiny::nesting::to_nesting_request(balanced.value());
  const auto maximum_request =
      shiny::nesting::to_nesting_request(maximum.value());

  REQUIRE(balanced_request.ok());
  REQUIRE(maximum_request.ok());
  REQUIRE(balanced_request.value().execution.strategy ==
          StrategyKind::metaheuristic_search);
  REQUIRE(balanced_request.value().execution.production_optimizer ==
          ProductionOptimizerKind::brkga);
  REQUIRE(maximum_request.value().execution.production.max_iterations >
          balanced_request.value().execution.production.max_iterations);
  REQUIRE(maximum_request.value().execution.production.population_size >
          balanced_request.value().execution.production.population_size);
}

TEST_CASE("profile request builder pins piece assignments through assigned bin id",
          "[api][profiles]") {
  const auto pinned = ProfileRequestBuilder{}
                          .with_profile(SolveProfile::quick)
                          .with_maintain_bed_assignment()
                          .add_bin(BinRequest{
                              .bin_id = 11,
                              .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
                          })
                          .add_bin(BinRequest{
                              .bin_id = 22,
                              .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
                          })
                          .add_piece(PieceRequest{
                              .piece_id = 201,
                              .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
                              .assigned_bin_id = 11,
                          })
                          .add_piece(PieceRequest{
                              .piece_id = 202,
                              .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
                              .assigned_bin_id = 22,
                          })
                          .build_checked();
  REQUIRE(pinned.ok());

  const auto translated = shiny::nesting::to_nesting_request(pinned.value());
  REQUIRE(translated.ok());
  REQUIRE(translated.value().pieces.back().allowed_bin_ids ==
          std::vector<std::uint32_t>{22U});

  const auto invalid = simple_profile_request(SolveProfile::quick)
                           .with_maintain_bed_assignment()
                           .build_checked();
  REQUIRE_FALSE(invalid.ok());
}

TEST_CASE("profile solve control builder round-trips request and control through the DTO surface",
          "[api][dto]") {
  const auto request = heavy_profile_request(SolveProfile::balanced, 1U);
  const auto control = ProfileSolveControlBuilder{}.with_random_seed(7).build();

  const auto request_dto = shiny::nesting::api::to_dto(request, control);
  const auto roundtrip_request = shiny::nesting::api::to_request(request_dto);
  const auto roundtrip_control =
      shiny::nesting::api::to_solve_control(request_dto.control);

  REQUIRE(roundtrip_request.profile == SolveProfile::balanced);
  REQUIRE(roundtrip_request.time_limit_milliseconds == 1U);
  REQUIRE(roundtrip_control.random_seed == 7U);

  const auto solved =
      shiny::nesting::solve(roundtrip_request, roundtrip_control);
  REQUIRE(solved.ok());
  REQUIRE(solved.value().stop_reason == StopReason::time_limit_reached);

  const auto active_bin_id =
      solved.value().layout.bins.empty()
          ? std::nullopt
          : std::optional<std::uint32_t>(
                solved.value().layout.bins.front().bin_id);
  const auto progress_dto = shiny::nesting::api::to_dto(ProfileProgressSnapshot{
      .profile = SolveProfile::balanced,
      .phase = shiny::nesting::ProgressPhase::completed,
      .phase_detail = "balanced",
      .current_layout = solved.value().layout,
      .best_layout = solved.value().layout,
      .active_bin_id = active_bin_id,
      .bin_summary = shiny::nesting::runtime::summarize_bins(
          solved.value().layout, active_bin_id),
      .placed_count = solved.value().placed_parts(),
      .utilization_percent = solved.value().utilization_percent(),
      .elapsed_time_milliseconds = 1U,
      .remaining_time_milliseconds = 0U,
      .stop_reason = solved.value().stop_reason,
      .improved = true,
  });
  REQUIRE(progress_dto.profile == SolveProfile::balanced);
  REQUIRE(progress_dto.phase == shiny::nesting::ProgressPhase::completed);
  REQUIRE(progress_dto.phase_detail == "balanced");
  REQUIRE(progress_dto.current_layout.placement_count ==
          solved.value().placed_parts());
  REQUIRE(progress_dto.bin_summary.size() == solved.value().layout.bins.size());
  REQUIRE(progress_dto.stop_reason == StopReason::time_limit_reached);
}

TEST_CASE(
    "quick profile with oversized pieces records partial placement",
    "[api][profiles]") {
  // 3×3 bin, two 4×4 pieces — neither can fit; overflow disabled so both
  // remain unplaced.  Verifies the partial-placement contract: valid layout,
  // completed stop reason, non-empty unplaced list.
  const auto request = ProfileRequestBuilder{}
                           .with_profile(SolveProfile::quick)
                           .with_allow_part_overflow(false)
                           .add_bin(BinRequest{
                               .bin_id = 1,
                               .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
                           })
                           .add_piece(PieceRequest{
                               .piece_id = 1,
                               .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
                           })
                           .add_piece(PieceRequest{
                               .piece_id = 2,
                               .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
                           })
                           .build_checked();
  REQUIRE(request.ok());

  const auto result = shiny::nesting::solve(
      request.value(), ProfileSolveControlBuilder{}.build());
  REQUIRE(result.ok());
  REQUIRE(result.value().layout_valid());
  REQUIRE(result.value().stop_reason == StopReason::completed);
  REQUIRE_FALSE(result.value().all_parts_placed());
  REQUIRE(result.value().unplaced_parts() == 2U);
}

TEST_CASE(
    "balanced profile places no fewer pieces than quick on the same input",
    "[api][profiles]") {
  // 12×12 bin, four 3×3 pieces — all trivially fit under both profiles.
  // Asserts the minimum quality ordering contract: balanced >= quick.
  const auto build_request = [](const SolveProfile profile,
                                std::optional<std::uint64_t> time_limit_ms)
      -> shiny::nesting::ProfileRequest {
    auto builder = ProfileRequestBuilder{};
    builder.with_profile(profile)
        .add_bin(BinRequest{
            .bin_id = 1,
            .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
        });
    for (std::uint32_t id = 1U; id <= 4U; ++id) {
      builder.add_piece(PieceRequest{
          .piece_id = id,
          .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      });
    }
    if (time_limit_ms.has_value()) {
      builder.with_time_limit_ms(*time_limit_ms);
    }
    const auto built = builder.build_checked();
    REQUIRE(built.ok());
    return built.value();
  };

  // operation_limit bounds the quick (bounding_box) solve.
  // BRKGA for balanced also respects it but will complete within max_iterations=8
  // first for a 4-piece 12x12 trivial fit anyway.
  const auto control =
      ProfileSolveControlBuilder{}.with_random_seed(17U).with_operation_limit(100U).build();

  const auto quick_result =
      shiny::nesting::solve(build_request(SolveProfile::quick, std::nullopt), control);
  REQUIRE(quick_result.ok());
  REQUIRE(quick_result.value().layout_valid());

  const auto balanced_result = shiny::nesting::solve(
      build_request(SolveProfile::balanced, 300U), control);
  REQUIRE(balanced_result.ok());
  REQUIRE(balanced_result.value().layout_valid());

  REQUIRE(balanced_result.value().placed_parts() >=
          quick_result.value().placed_parts());
}

TEST_CASE(
    "public headers and README document the full profile workflow surface",
    "[api][docs]") {
  const auto request_header = read_text(repo_root() / "src/request.hpp");
  const auto builder_header =
      read_text(repo_root() / "src/api/request_builder.hpp");
  const auto dto_header = read_text(repo_root() / "src/api/dto.hpp");
  const auto readme = read_text(repo_root() / "README.md");

  REQUIRE(request_header.find("enum class SolveProfile") != std::string::npos);
  REQUIRE(request_header.find("assigned_bin_id") != std::string::npos);
  REQUIRE(builder_header.find("class ProfileRequestBuilder") !=
          std::string::npos);
  REQUIRE(builder_header.find("class ProfileSolveControlBuilder") !=
          std::string::npos);
  REQUIRE(builder_header.find("with_maintain_bed_assignment") !=
          std::string::npos);
  REQUIRE(dto_header.find("ProfileProgressSnapshotDto") != std::string::npos);
  REQUIRE(dto_header.find("BinProgressSummaryDto") != std::string::npos);
  REQUIRE(readme.find("ProfileRequestBuilder") != std::string::npos);
  REQUIRE(readme.find("api/solve_control.hpp") != std::string::npos);
  REQUIRE(readme.find("SolveProfile::quick") != std::string::npos);
  REQUIRE(readme.find("SolveProfile::balanced") != std::string::npos);
  REQUIRE(readme.find("SolveProfile::maximum_search") != std::string::npos);
  REQUIRE(readme.find("bin_summary") != std::string::npos);
  REQUIRE(readme.find("export_face_migration_checklist.md") !=
          std::string::npos);
  REQUIRE(readme.find("ProductionOptimizerKind::brkga") == std::string::npos);
}