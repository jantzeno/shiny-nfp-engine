#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "support/fixture_test_support.hpp"

namespace {

namespace fs = std::filesystem;

struct CoverageOwner {
  std::string_view surface{};
  fs::path relative_path{};
  std::vector<std::string_view> required_substrings{};
};

auto repo_root() -> fs::path {
  return shiny::nesting::test::fixture_root().parent_path().parent_path();
}

auto read_text(const fs::path &path) -> std::string {
  std::ifstream input(path);
  REQUIRE(input.is_open());
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

} // namespace

TEST_CASE("public request surface ownership manifest stays visible",
          "[manifest][mtg][nesting-matrix]") {
  const std::vector<CoverageOwner> owners = {
      {
          .surface = "bounding-box algorithm readiness",
          .relative_path = "tests/integration/mtg_test_nesting_matrix.cpp",
          .required_substrings =
              {
                  "mtg baseline bounding-box solve places every part",
              },
      },
      {
          .surface = "sequential-backtrack algorithm readiness",
          .relative_path = "tests/integration/mtg_test_nesting_matrix.cpp",
          .required_substrings =
              {
                  "mtg baseline sequential-backtrack solve places every part",
              },
      },
      {
          .surface = "metaheuristic-search algorithm breadth",
          .relative_path = "tests/integration/metaheuristic_search.cpp",
          .required_substrings =
              {
                  "mtg metaheuristic-search positive matrix places every part "
                  "on the ",
                  "asymmetric synthetic fixture",
              },
      },
      {
          .surface = "production optimizer breadth",
          .relative_path = "tests/integration/metaheuristic_search.cpp",
          .required_substrings =
              {
                  "ProductionOptimizerKind::brkga",
                  "ProductionOptimizerKind::simulated_annealing",
                  "ProductionOptimizerKind::alns",
                  "ProductionOptimizerKind::gdrr",
                  "ProductionOptimizerKind::lahc",
              },
      },
      {
          .surface = "candidate-strategy actual-polygon readiness",
          .relative_path = "tests/integration/mtg_engine_bug_repros.cpp",
          .required_substrings =
              {
                  "CandidateStrategy::anchor_vertex",
                  "CandidateStrategy::nfp_perfect",
                  "CandidateStrategy::nfp_arrangement",
                  "CandidateStrategy::nfp_hybrid",
                  "REGRESSION: anchor_vertex places every actual-polygon MTG "
                  "silhouette",
                  "REGRESSION: NFP strategies place every actual-polygon MTG "
                  "silhouette",
              },
      },
      {
          .surface = "piece-ordering breadth",
          .relative_path = "tests/integration/sequential_backtrack.cpp",
          .required_substrings =
              {
                  "PieceOrdering::input",
                  "PieceOrdering::largest_area_first",
                  "PieceOrdering::hull_diameter_first",
                  "PieceOrdering::priority",
              },
      },
      {
          .surface = "selected bins and allowed-bin restrictions",
          .relative_path = "tests/integration/bounding_box_engine_surface.cpp",
          .required_substrings =
              {
                  "selected_bin_ids",
                  "allowed_bin_ids",
                  "focused asymmetric fixture honors non-conflicting "
                  "allowed_bin_ids",
              },
      },
      {
          .surface = "rotation controls",
          .relative_path = "tests/integration/bounding_box_engine_surface.cpp",
          .required_substrings =
              {
                  "default_rotations",
                  "allowed_rotations",
                  "focused asymmetric fixture makes default_rotations directly "
                  "observable",
              },
      },
      {
          .surface = "mirror toggle",
          .relative_path = "tests/integration/bounding_box_engine_surface.cpp",
          .required_substrings =
              {
                  "mtg allow_mirror toggle preserves placement count",
              },
      },
      {
          .surface = "quantity expansion",
          .relative_path = "tests/integration/sequential_backtrack.cpp",
          .required_substrings =
              {
                  "mtg sequential-backtrack expands quantity>1 instances",
              },
      },
      {
          .surface = "maintain-bed-assignment behavior",
          .relative_path = "tests/integration/metaheuristic_search.cpp",
          .required_substrings =
              {
                  "mtg metaheuristic-search maintain_bed_assignment pins "
                  "pieces to source ",
                  "beds",
              },
      },
      {
          .surface = "part spacing slider coverage",
          .relative_path = "tests/integration/mtg_sliders.cpp",
          .required_substrings =
              {
                  "mtg slider part_spacing_mm bounding-box (max)",
                  "mtg slider part_spacing_mm sequential-backtrack (max)",
              },
      },
      {
          .surface = "start-corner overrides",
          .relative_path = "tests/integration/mtg_start_corners.cpp",
          .required_substrings =
              {
                  "mtg start-corner override matrix (sequential_backtrack)",
              },
      },
      {
          .surface = "bed margins",
          .relative_path = "tests/integration/mtg_margins.cpp",
          .required_substrings =
              {
                  "mtg uniform bed margins still place every part",
                  "mtg asymmetric bed margins respect each side",
              },
      },
      {
          .surface = "exclusion zones",
          .relative_path = "tests/integration/mtg_exclusion_zones.cpp",
          .required_substrings =
              {
                  "mtg exclusion zone on bed1 forces overflow to bed2",
              },
      },
      {
          .surface = "grain compatibility normalization",
          .relative_path = "tests/unit/request_normalization.cpp",
          .required_substrings =
              {
                  "grain_compatibility = "
                  "PartGrainCompatibility::parallel_to_bed",
                  "pieces.front().grain_compatibility ==",
              },
      },
      {
          .surface = "part-in-part placement",
          .relative_path = "tests/integration/sequential_backtrack.cpp",
          .required_substrings =
              {
                  "sequential-backtrack enable_part_in_part_placement fills "
                  "the hole",
              },
      },
      {
          .surface = "concave-candidate exploration",
          .relative_path = "tests/integration/metaheuristic_search.cpp",
          .required_substrings =
              {
                  "mtg metaheuristic-search explore_concave_candidates still "
                  "places ",
                  "every part on the asymmetric synthetic fixture",
              },
      },
      {
          .surface = "priority ordering visibility",
          .relative_path = "tests/integration/bounding_box_engine_surface.cpp",
          .required_substrings =
              {
                  "mtg piece priority survives partial placement",
              },
      },
  };

  std::unordered_map<std::string, std::string> cache;

  for (const auto &owner : owners) {
    const auto absolute_path = repo_root() / owner.relative_path;
    auto [it, inserted] =
        cache.emplace(owner.relative_path.string(), std::string{});
    if (inserted) {
      it->second = read_text(absolute_path);
    }

    DYNAMIC_SECTION(owner.surface) {
      INFO("owner file: " << owner.relative_path.string());
      for (const auto substring : owner.required_substrings) {
        INFO("required substring: " << substring);
        REQUIRE(it->second.find(substring) != std::string::npos);
      }
    }
  }
}
