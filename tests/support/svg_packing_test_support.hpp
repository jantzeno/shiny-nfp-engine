#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "packing/decoder.hpp"
#include "packing/masonry.hpp"
#include "search/request.hpp"
#include "search/result.hpp"
#include "util/status.hpp"

namespace shiny::nfp::test::svg {

inline constexpr double kMinimumArea = 1e-6;
inline constexpr std::uint32_t kSearchSeed = 7;

struct SvgPackingCaseSpec {
  std::string id{};
  std::string description{};
  std::string source{"handmade"};
  std::string acceptance_lane{"readiness"};
  std::vector<std::string> coverage_features{};
  std::vector<std::string> algorithm_applicability{"jostle_search"};
  std::filesystem::path input_path{};
  bool normative{true};
  bool slow_exploratory{false};
  bool require_observable_success_before_interrupt{false};
  bool require_explicit_bed_id{true};
  std::size_t max_candidate_groups{6};
  std::size_t max_piece_count{0};
  double min_piece_area{kMinimumArea};
  std::vector<double> allowed_rotations_degrees{0.0};
  place::BedGrainDirection bed_grain_direction{
      place::BedGrainDirection::unrestricted};
  std::vector<place::BedExclusionZone> exclusion_zones{};
  std::unordered_map<std::string, place::PartGrainCompatibility>
      piece_grain_compatibility{};
  std::size_t max_bin_count{1};
  std::size_t min_placed_piece_count{1};
  bool require_full_placement{false};
  std::uint32_t search_iterations{1};
  std::uint32_t search_plateau_budget{1};
  std::uint32_t execution_time_budget_ms{0};
};

struct ImportedPiece {
  std::string key{};
  geom::PolygonWithHoles polygon{};
  double area{0.0};
};

struct ImportedSvgCase {
  std::filesystem::path source_path{};
  geom::PolygonWithHoles bed{};
  std::vector<ImportedPiece> pieces{};
};

[[nodiscard]] auto output_root() -> std::filesystem::path;

[[nodiscard]] auto load_svg_case_specs(bool normative_only)
    -> std::vector<SvgPackingCaseSpec>;

[[nodiscard]] auto case_supports_algorithm(const SvgPackingCaseSpec &spec,
                                           std::string_view algorithm) -> bool;

[[nodiscard]] auto select_svg_case_specs(bool normative_only,
                                         std::string_view algorithm)
    -> std::vector<SvgPackingCaseSpec>;

[[nodiscard]] auto import_svg_case(const SvgPackingCaseSpec &spec)
    -> util::StatusOr<ImportedSvgCase>;

[[nodiscard]] auto make_decoder_request(const SvgPackingCaseSpec &spec,
                                        const ImportedSvgCase &svg_case)
    -> pack::DecoderRequest;

[[nodiscard]] auto make_search_request(const SvgPackingCaseSpec &spec,
                                       const ImportedSvgCase &svg_case)
    -> search::SearchRequest;

[[nodiscard]] auto make_masonry_request(const SvgPackingCaseSpec &spec,
                                        const ImportedSvgCase &svg_case)
    -> pack::MasonryRequest;

[[nodiscard]] auto write_layout_svg(const std::filesystem::path &path,
                                    const pack::Layout &layout) -> util::Status;

void require_valid_imported_case(const SvgPackingCaseSpec &spec,
                                 const ImportedSvgCase &imported);

void require_request_matches_imported_case(const SvgPackingCaseSpec &spec,
                                           const ImportedSvgCase &imported,
                                           const pack::DecoderRequest &request);

void require_valid_decoder_result(const SvgPackingCaseSpec &spec,
                                  const pack::DecoderRequest &request,
                                  const pack::DecoderResult &result);

void require_valid_search_result(const SvgPackingCaseSpec &spec,
                                 const search::SearchRequest &request,
                                 const search::SearchResult &result);

void require_valid_masonry_result(const SvgPackingCaseSpec &spec,
                                  const pack::MasonryRequest &request,
                                  const pack::MasonryResult &result);

void require_same_layout(const pack::Layout &lhs, const pack::Layout &rhs);

void require_same_decoder_result(const pack::DecoderResult &lhs,
                                 const pack::DecoderResult &rhs);

void require_same_search_result(const search::SearchResult &lhs,
                                const search::SearchResult &rhs);

void require_same_masonry_result(const pack::MasonryResult &lhs,
                                 const pack::MasonryResult &rhs);

} // namespace shiny::nfp::test::svg