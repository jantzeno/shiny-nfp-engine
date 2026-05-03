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
          .surface = "profile-first builder surface",
          .relative_path = "tests/unit/api/profiles.cpp",
          .required_substrings =
              {
                  "ProfileRequestBuilder",
                  "ProfileSolveControlBuilder",
                  "SolveControlBuilder",
              },
      },
      {
          .surface = "profile workflow coverage",
          .relative_path = "tests/unit/api/profiles.cpp",
          .required_substrings =
              {
                  "SolveProfile::quick",
                  "SolveProfile::balanced",
                  "SolveProfile::maximum_search",
                  "ProfileProgressSnapshotDto",
              },
      },
      {
          .surface = "selected bins and allowed-bin restrictions",
          .relative_path = "tests/integration/export_surface/"
                           "mtg_bounding_box_engine_surface.cpp",
          .required_substrings =
              {
                  "selected_bin_ids",
                  "allowed_bin_ids",
                  "focused asymmetric fixture honors non-conflicting "
                  "allowed_bin_ids",
              },
      },
      {
          .surface = "overflow and assignment contract",
          .relative_path =
              "tests/integration/export_surface/mtg_bin_assignment.cpp",
          .required_substrings =
              {
                  "allow_part_overflow",
                  "maintain_bed_assignment",
                  "maintain_bed_assignment × allow_part_overflow truth table",
              },
      },
      {
          .surface = "balanced profile integration",
          .relative_path = "tests/integration/profiles/balanced.cpp",
          .required_substrings =
              {
                  "profile_balanced",
                  "remaining_time_milliseconds",
                  "balanced Sparrow progress",
              },
      },
      {
          .surface = "maximum-search profile integration",
          .relative_path = "tests/integration/profiles/maximum_search.cpp",
          .required_substrings =
              {
                  "profile_maximum_search",
                  "maximum-search",
                  "remaining_time_milliseconds",
              },
      },
      {
          .surface = "multi-bin integration",
          .relative_path = "tests/integration/multi_bin.cpp",
          .required_substrings =
              {
                  "assignment-overflow",
                  "active_bin_ids",
                  "ProfileProgressSnapshot",
              },
      },
      {
          .surface = "knapsack integration",
          .relative_path = "tests/integration/knapsack.cpp",
          .required_substrings =
              {
                  "maximize_value",
                  "knapsack",
                  "higher-value subset",
              },
      },
      {
          .surface = "README public workflow",
          .relative_path = "README.md",
          .required_substrings =
              {
                  "ProfileRequestBuilder",
                  "SolveProfile::quick",
                  "SolveProfile::balanced",
                  "SolveProfile::maximum_search",
                  "bin_summary",
                  "src/runtime/progress.hpp",
                  "export_face_migration_checklist.md",
              },
      },
      {
          .surface = "downstream migration checklist",
          .relative_path = "docs/export_face_migration_checklist.md",
          .required_substrings =
              {
                  "request construction",
                  "Remove strategy-specific UI",
                  "Wire the new progress DTOs",
                  "overflow and assignment semantics",
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

  const auto request_header = read_text(repo_root() / "src/request.hpp");
  DYNAMIC_SECTION("public request header keeps normalization wrapper only") {
    REQUIRE(request_header.find("normalize_nesting_request") !=
            std::string::npos);
    REQUIRE(request_header.find("NormalizedRequest") == std::string::npos);
    REQUIRE(request_header.find("ExpandedPieceInstance") == std::string::npos);
    REQUIRE(request_header.find("ExpandedBinInstance") == std::string::npos);
  }
}
