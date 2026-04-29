#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/transform.hpp"
#include "io/json.hpp"
#include "io/layout_svg.hpp"
#include "io/or_dataset_json.hpp"
#include "solve.hpp"
#include "support/fixture_test_support.hpp"

namespace {

namespace fs = std::filesystem;
namespace pt = boost::property_tree;

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::ProductionSearchConfig;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::DiscreteRotationSet;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::io::OrDataset;

struct ReadinessCase {
  std::string id{};
  NestingRequest request{};
};

struct ReadinessObservation {
  std::string id{};
  std::string strategy{"brkga"};
  std::uint64_t seed{17};
  std::size_t placed_parts{0};
  double utilization{0.0};
  std::uint64_t elapsed_milliseconds{0};
  std::size_t fallback_candidates{0};
  std::size_t selected_fallback_placements{0};
  StopReason stop_reason{StopReason::none};
  bool full_success{false};
  bool layout_valid{false};
  std::size_t validation_issue_count{0};
  std::string layout_json_artifact{};
  std::string layout_svg_artifact{};
};

struct ReadinessOptions {
  std::vector<std::string> case_ids{};
  std::vector<std::string> strategies{"brkga"};
  std::uint64_t seed{17};
  std::size_t repeat{1};
  fs::path artifact_dir{fs::current_path() / "artifacts" / "readiness"};
  bool compare_baselines{true};
  bool refresh_baselines{false};
};

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return {
      .outer =
          {
              {min_x, min_y},
              {max_x, min_y},
              {max_x, max_y},
              {min_x, max_y},
              {min_x, min_y},
          },
  };
}

auto normalize_origin(const PolygonWithHoles &polygon) -> PolygonWithHoles {
  const auto bounds = shiny::nesting::geom::compute_bounds(polygon);
  return shiny::nesting::geom::normalize_polygon(shiny::nesting::geom::translate(
      polygon, {.x = -bounds.min.x, .y = -bounds.min.y}));
}

auto readiness_request_base() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.enable_part_in_part_placement = true;
  request.execution.irregular.enable_direct_overlap_check = true;
  request.execution.default_rotations = DiscreteRotationSet{
      .angles_degrees = {0.0, 90.0, 180.0, 270.0}};
  request.execution.production_optimizer = ProductionOptimizerKind::brkga;
  request.execution.production = ProductionSearchConfig{};
  request.execution.production.population_size = 10;
  request.execution.production.elite_count = 3;
  request.execution.production.mutant_count = 2;
  request.execution.production.max_iterations = 6;
  request.execution.production.diversification_swaps = 1;
  request.execution.production.polishing_passes = 1;
  request.execution.production.strip_exploration_ratio = 0.75;
  request.execution.production.separator_worker_count = 1;
  request.execution.production.separator_max_iterations = 32;
  request.execution.production.separator_iter_no_improvement_limit = 6;
  request.execution.production.separator_strike_limit = 3;
  request.execution.production.separator_global_samples = 24;
  request.execution.production.separator_focused_samples = 12;
  request.execution.production.separator_coordinate_descent_iterations = 16;
  return request;
}

auto parse_points(std::string points_text) -> shiny::nesting::geom::Ring {
  for (auto &ch : points_text) {
    if (ch == ',') {
      ch = ' ';
    }
  }

  std::stringstream stream(points_text);
  shiny::nesting::geom::Ring ring;
  double x = 0.0;
  double y = 0.0;
  while (stream >> x >> y) {
    ring.push_back({x, y});
  }
  if (!ring.empty() && ring.front() != ring.back()) {
    ring.push_back(ring.front());
  }
  return ring;
}

auto request_from_svg_fixture(const fs::path &path) -> NestingRequest {
  const auto polygons = [&]() {
    std::ifstream input(path);
    REQUIRE(input.is_open());
    const std::string svg((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());

    const std::regex polygon_pattern(R"svg(<polygon\b([^>]*)/?>)svg");
    const std::regex id_pattern(R"svg(id="([^"]+)")svg");
    const std::regex points_pattern(R"svg(points="([^"]+)")svg");
    std::sregex_iterator it(svg.begin(), svg.end(), polygon_pattern);
    std::sregex_iterator end;

    std::vector<std::pair<std::string, PolygonWithHoles>> parsed;
    for (; it != end; ++it) {
      const auto attributes = (*it)[1].str();
      std::smatch id_match;
      std::smatch points_match;
      if (!std::regex_search(attributes, id_match, id_pattern) ||
          !std::regex_search(attributes, points_match, points_pattern)) {
        continue;
      }

      PolygonWithHoles polygon{.outer = parse_points(points_match[1].str())};
      parsed.push_back(std::pair<std::string, PolygonWithHoles>{
          id_match[1].str(),
          shiny::nesting::geom::normalize_polygon(polygon),
      });
    }
    return parsed;
  }();
  REQUIRE(polygons.size() >= 2U);

  auto request = readiness_request_base();
  const auto bed_it = std::find_if(polygons.begin(), polygons.end(),
                                   [](const auto &entry) {
                                     return entry.first == "bed";
                                   });
  REQUIRE(bed_it != polygons.end());
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = normalize_origin(bed_it->second),
  });

  std::uint32_t piece_id = 100;
  for (const auto &[id, polygon] : polygons) {
    if (id == "bed") {
      continue;
    }
    request.pieces.push_back(PieceRequest{
        .piece_id = piece_id++,
        .polygon = normalize_origin(polygon),
        .allowed_rotations = DiscreteRotationSet{.angles_degrees = {0.0}},
    });
  }
  return request;
}

auto parse_svg_fixture(const fs::path &path)
    -> std::vector<std::pair<std::string, PolygonWithHoles>> {
  std::ifstream input(path);
  REQUIRE(input.is_open());
  const std::string svg((std::istreambuf_iterator<char>(input)),
                        std::istreambuf_iterator<char>());

  const std::regex polygon_pattern(R"svg(<polygon\b([^>]*)/?>)svg");
  const std::regex id_pattern(R"svg(id="([^"]+)")svg");
  const std::regex points_pattern(R"svg(points="([^"]+)")svg");
  std::sregex_iterator it(svg.begin(), svg.end(), polygon_pattern);
  std::sregex_iterator end;

  std::vector<std::pair<std::string, PolygonWithHoles>> polygons;
  for (; it != end; ++it) {
    const auto attributes = (*it)[1].str();
    std::smatch id_match;
    std::smatch points_match;
    if (!std::regex_search(attributes, id_match, id_pattern) ||
        !std::regex_search(attributes, points_match, points_pattern)) {
      continue;
    }

    PolygonWithHoles polygon{.outer = parse_points(points_match[1].str())};
    polygons.push_back(std::pair<std::string, PolygonWithHoles>{
        id_match[1].str(),
        shiny::nesting::geom::normalize_polygon(polygon),
    });
  }
  return polygons;
}

auto request_from_or_dataset(const OrDataset &dataset) -> NestingRequest {
  auto request = readiness_request_base();
  request.pieces.clear();
  request.bins.clear();

  std::uint32_t next_piece_id = 100;
  for (const auto &item : dataset.items) {
    request.pieces.push_back(PieceRequest{
        .piece_id = next_piece_id++,
        .polygon = normalize_origin(item.polygon),
        .quantity = item.demand,
        .allowed_rotations = item.allowed_orientations.empty()
                                 ? std::nullopt
                                 : std::optional<DiscreteRotationSet>{
                                       {.angles_degrees =
                                            item.allowed_orientations}},
    });
  }

  if (dataset.uses_explicit_bins()) {
    for (const auto &bin : dataset.bins) {
      request.bins.push_back(BinRequest{
          .bin_id = bin.bin_id,
          .polygon = normalize_origin(bin.polygon),
          .stock = bin.stock,
      });
    }
    return request;
  }

  REQUIRE(dataset.strip_height.has_value());
  double strip_width = 0.0;
  for (const auto &item : dataset.items) {
    const auto bounds = shiny::nesting::geom::compute_bounds(item.polygon);
    strip_width += shiny::nesting::geom::box_width(bounds) * item.demand;
  }
  strip_width = std::max(strip_width * 1.5, 12.0);
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, strip_width, *dataset.strip_height),
  });
  return request;
}

auto readiness_cases() -> std::vector<ReadinessCase> {
  const auto fixtures = shiny::nesting::test::fixture_root();
  const auto svg_fixtures = fixtures.parent_path() / "files" / "tests";
  const auto strip_dataset =
      shiny::nesting::io::load_or_dataset(fixtures / "or_datasets/strip_dataset.json");
  REQUIRE(strip_dataset.ok());
  const auto explicit_bins = shiny::nesting::io::load_or_dataset(
      fixtures / "or_datasets/explicit_bins_dataset.json");
  REQUIRE(explicit_bins.ok());

  return {
      {
          .id = "or-strip-dataset",
          .request = request_from_or_dataset(strip_dataset.value()),
      },
      {
          .id = "or-explicit-bins",
          .request = request_from_or_dataset(explicit_bins.value()),
      },
      {
          .id = "svg-full-irregular-set",
          .request =
              request_from_svg_fixture(svg_fixtures / "readiness_full_irregular_set.svg"),
      },
  };
}

auto baseline_path() -> fs::path {
  return shiny::nesting::test::fixture_root().parent_path() / "readiness" /
         "baselines" / "readiness_matrix.json";
}

auto stop_reason_name(const StopReason reason) -> std::string_view {
  switch (reason) {
  case StopReason::none:
    return "none";
  case StopReason::completed:
    return "completed";
  case StopReason::time_limit_reached:
    return "time_limit_reached";
  case StopReason::operation_limit_reached:
    return "operation_limit_reached";
  case StopReason::cancelled:
    return "cancelled";
  case StopReason::invalid_request:
    return "invalid_request";
  }
  return "unknown";
}

auto split_csv(std::string_view text) -> std::vector<std::string> {
  std::vector<std::string> values;
  std::size_t start = 0;
  while (start <= text.size()) {
    const auto comma = text.find(',', start);
    const auto end = comma == std::string_view::npos ? text.size() : comma;
    if (end > start) {
      values.emplace_back(text.substr(start, end - start));
    }
    if (comma == std::string_view::npos) {
      break;
    }
    start = comma + 1U;
  }
  return values;
}

auto env_u64(const char *name, const std::uint64_t fallback) -> std::uint64_t {
  const auto *value = std::getenv(name);
  return value == nullptr ? fallback : std::stoull(value);
}

auto env_size(const char *name, const std::size_t fallback) -> std::size_t {
  return static_cast<std::size_t>(env_u64(name, fallback));
}

auto env_flag(const char *name) -> bool {
  const auto *value = std::getenv(name);
  return value != nullptr && std::string_view(value) == "1";
}

auto readiness_options() -> ReadinessOptions {
  ReadinessOptions options;
  options.artifact_dir = shiny::nesting::test::fixture_root().parent_path()
                             .parent_path() /
                         "artifacts" / "readiness";
  if (const auto *cases = std::getenv("SHINY_NESTING_ENGINE_READINESS_CASES");
      cases != nullptr) {
    options.case_ids = split_csv(cases);
  }
  if (const auto *strategies =
          std::getenv("SHINY_NESTING_ENGINE_READINESS_STRATEGIES");
      strategies != nullptr) {
    options.strategies = split_csv(strategies);
  }
  options.seed = env_u64("SHINY_NESTING_ENGINE_READINESS_SEED", options.seed);
  options.repeat =
      std::max<std::size_t>(1U, env_size("SHINY_NESTING_ENGINE_READINESS_REPEAT",
                                         options.repeat));
  if (const auto *output = std::getenv("SHINY_NESTING_ENGINE_READINESS_OUTPUT");
      output != nullptr) {
    options.artifact_dir = output;
  }
  options.compare_baselines =
      !env_flag("SHINY_NESTING_ENGINE_NO_BASELINE_COMPARE");
  options.refresh_baselines =
      env_flag("SHINY_NESTING_ENGINE_REFRESH_BASELINES");
  return options;
}

auto production_optimizer_from_name(std::string_view name)
    -> ProductionOptimizerKind {
  if (name == "simulated_annealing") {
    return ProductionOptimizerKind::simulated_annealing;
  }
  if (name == "alns") {
    return ProductionOptimizerKind::alns;
  }
  if (name == "gdrr") {
    return ProductionOptimizerKind::gdrr;
  }
  if (name == "lahc") {
    return ProductionOptimizerKind::lahc;
  }
  return ProductionOptimizerKind::brkga;
}

auto observation_to_node(const ReadinessObservation &observation) -> pt::ptree {
  pt::ptree node;
  node.put("min_placed_parts", observation.placed_parts);
  node.put("min_utilization", observation.utilization);
  node.put("max_runtime_ms",
           std::max<std::uint64_t>(50U,
                                   observation.elapsed_milliseconds * 4U + 25U));
  node.put("max_fallback_candidates", observation.fallback_candidates + 1U);
  node.put("max_selected_fallback_placements",
           observation.selected_fallback_placements + 1U);
  node.put("expected_stop_reason",
           std::string(stop_reason_name(observation.stop_reason)));
  node.put("require_full_success", observation.full_success);
  node.put("require_layout_valid", observation.layout_valid);
  node.put("max_validation_issues", observation.validation_issue_count);
  return node;
}

auto load_baselines() -> pt::ptree {
  pt::ptree root;
  pt::read_json(baseline_path().string(), root);
  return root;
}

void maybe_refresh_baselines(
    const std::vector<ReadinessObservation> &observations,
    const ReadinessOptions &options) {
  if (!options.refresh_baselines) {
    return;
  }

  pt::ptree root;
  root.put("schema_version", 1);
  pt::ptree cases_node;
  for (const auto &observation : observations) {
    cases_node.add_child(observation.id, observation_to_node(observation));
  }
  root.add_child("cases", cases_node);

  fs::create_directories(baseline_path().parent_path());
  pt::write_json(baseline_path().string(), root);
}

void write_results_json(const std::vector<ReadinessObservation> &observations,
                        const fs::path &path) {
  fs::create_directories(path.parent_path());
  std::ofstream output(path);
  REQUIRE(output.is_open());
  output << std::boolalpha;
  output << "{\n  \"schema_version\": 1,\n  \"records\": [\n";
  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto &observation = observations[index];
    output << "    {\n"
           << "      \"case_id\": \"" << observation.id << "\",\n"
           << "      \"strategy\": \"" << observation.strategy << "\",\n"
           << "      \"seed\": " << observation.seed << ",\n"
           << "      \"placed_parts\": " << observation.placed_parts << ",\n"
           << "      \"utilization_percent\": " << observation.utilization << ",\n"
           << "      \"runtime_ms\": " << observation.elapsed_milliseconds
           << ",\n"
           << "      \"fallback_candidates\": " << observation.fallback_candidates
           << ",\n"
           << "      \"selected_fallback_placements\": "
           << observation.selected_fallback_placements << ",\n"
            << "      \"stop_reason\": \""
            << stop_reason_name(observation.stop_reason) << "\",\n"
            << "      \"full_success\": " << observation.full_success << ",\n"
            << "      \"layout_valid\": " << observation.layout_valid << ",\n"
            << "      \"validation_issue_count\": "
            << observation.validation_issue_count << ",\n"
            << "      \"layout_json_artifact\": \""
            << observation.layout_json_artifact << "\",\n"
            << "      \"layout_svg_artifact\": \""
            << observation.layout_svg_artifact << "\"\n"
            << "    }" << (index + 1U == observations.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
}

auto read_text_file(const fs::path &path) -> std::string {
  std::ifstream input(path);
  REQUIRE(input.is_open());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

auto run_readiness_case(const ReadinessCase &test_case, std::string_view strategy,
                        const std::uint64_t seed, const fs::path &artifact_dir)
    -> ReadinessObservation {
  CAPTURE(test_case.id);
  CAPTURE(strategy);
  CAPTURE(seed);
  auto request = test_case.request;
  request.execution.production_optimizer = production_optimizer_from_name(strategy);
  REQUIRE(request.is_valid());

  const auto started = std::chrono::steady_clock::now();
  const auto result = shiny::nesting::solve(
      request, SolveControl{.random_seed = seed});
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started);

  REQUIRE(result.ok());
  REQUIRE(result.value().layout_valid());

  const auto artifact_stem =
      test_case.id + "-" + std::string(strategy) + "-" + std::to_string(seed);
  const auto json_path = artifact_dir / (artifact_stem + ".json");
  const auto svg_path = artifact_dir / (artifact_stem + ".svg");
  REQUIRE(shiny::nesting::io::save_layout(json_path, result.value().layout) ==
          shiny::nesting::util::Status::ok);
  REQUIRE(shiny::nesting::io::save_layout_svg(svg_path, result.value().layout) ==
          shiny::nesting::util::Status::ok);

  const auto json_text = read_text_file(json_path);
  const auto svg_text = read_text_file(svg_path);
  REQUIRE(json_text.find("\"kind\": \"layout\"") != std::string::npos);
  REQUIRE(json_text.find("\"placements\"") != std::string::npos);
  REQUIRE(svg_text.find("<svg") != std::string::npos);
  REQUIRE(svg_text.find("bin-") != std::string::npos);
  REQUIRE(json_path.filename().string() == artifact_stem + ".json");
  REQUIRE(svg_path.filename().string() == artifact_stem + ".svg");

  return {
      .id = test_case.id,
      .strategy = std::string(strategy),
      .seed = seed,
      .placed_parts = result.value().placed_parts(),
      .utilization = result.value().summary().utilization_percent,
      .elapsed_milliseconds = static_cast<std::uint64_t>(elapsed.count()),
      .fallback_candidates =
          result.value().search.fallback_metrics.conservative_bbox_candidate_points,
      .selected_fallback_placements =
          result.value().search.fallback_metrics.selected_fallback_placements,
      .stop_reason = result.value().stop_reason,
      .full_success = result.value().is_full_success(),
      .layout_valid = result.value().layout_valid(),
      .validation_issue_count = result.value().validation.issues.size(),
      .layout_json_artifact = json_path.filename().string(),
      .layout_svg_artifact = svg_path.filename().string(),
  };
}

void require_within_baseline(const ReadinessObservation &observation,
                             const pt::ptree &baseline_root) {
  const auto baseline = baseline_root.get_child_optional("cases." + observation.id);
  REQUIRE(baseline.has_value());

  REQUIRE(observation.placed_parts >=
          baseline->get<std::size_t>("min_placed_parts"));
  REQUIRE(observation.utilization + 1e-9 >=
          baseline->get<double>("min_utilization"));
  REQUIRE(observation.elapsed_milliseconds <=
          baseline->get<std::uint64_t>("max_runtime_ms"));
  REQUIRE(observation.fallback_candidates <=
          baseline->get<std::size_t>("max_fallback_candidates"));
  REQUIRE(observation.selected_fallback_placements <=
          baseline->get<std::size_t>("max_selected_fallback_placements"));
  REQUIRE(stop_reason_name(observation.stop_reason) ==
          baseline->get<std::string>("expected_stop_reason"));
  if (baseline->get<bool>("require_full_success")) {
    REQUIRE(observation.full_success);
  }
  if (baseline->get<bool>("require_layout_valid", true)) {
    REQUIRE(observation.layout_valid);
  }
  REQUIRE(observation.validation_issue_count <=
          baseline->get<std::size_t>("max_validation_issues", 0U));
}

} // namespace

TEST_CASE("readiness matrix stays within baseline envelope",
           "[readiness][benchmark-smoke]") {
  const auto options = readiness_options();
  fs::remove_all(options.artifact_dir);
  fs::create_directories(options.artifact_dir);

  std::vector<ReadinessObservation> observations;
  for (const auto &test_case : readiness_cases()) {
    if (!options.case_ids.empty() &&
        std::find(options.case_ids.begin(), options.case_ids.end(),
                  test_case.id) == options.case_ids.end()) {
      continue;
    }
    for (const auto &strategy : options.strategies) {
      for (std::size_t repeat = 0; repeat < options.repeat; ++repeat) {
        observations.push_back(run_readiness_case(
            test_case, strategy, options.seed + repeat, options.artifact_dir));
      }
    }
  }
  REQUIRE_FALSE(observations.empty());
  write_results_json(observations, options.artifact_dir / "results.json");

  maybe_refresh_baselines(observations, options);
  if (options.compare_baselines) {
    const auto baselines = load_baselines();
    REQUIRE(baselines.get<int>("schema_version") == 1);
    for (const auto &observation : observations) {
      require_within_baseline(observation, baselines);
    }
  }

  pt::ptree results;
  pt::read_json((options.artifact_dir / "results.json").string(), results);
  REQUIRE(results.get<int>("schema_version") == 1);
  const auto &records = results.get_child("records");
  REQUIRE(std::distance(records.begin(), records.end()) ==
          static_cast<std::ptrdiff_t>(observations.size()));
  for (const auto &record_node : records) {
    const auto &record = record_node.second;
    const auto case_id = record.get<std::string>("case_id");
    const auto strategy = record.get<std::string>("strategy");
    const auto seed = record.get<std::uint64_t>("seed");
    const auto json_artifact = record.get<std::string>("layout_json_artifact");
    const auto svg_artifact = record.get<std::string>("layout_svg_artifact");
    REQUIRE_FALSE(case_id.empty());
    REQUIRE_FALSE(strategy.empty());
    REQUIRE(record.get<bool>("layout_valid"));
    REQUIRE(record.get<std::size_t>("validation_issue_count") == 0U);
    REQUIRE(json_artifact == case_id + "-" + strategy + "-" +
                                 std::to_string(seed) + ".json");
    REQUIRE(svg_artifact == case_id + "-" + strategy + "-" +
                                std::to_string(seed) + ".svg");
    REQUIRE(fs::path(json_artifact).is_relative());
    REQUIRE(fs::path(svg_artifact).is_relative());
    REQUIRE((options.artifact_dir / json_artifact).filename() == json_artifact);
    REQUIRE((options.artifact_dir / svg_artifact).filename() == svg_artifact);
  }
}

TEST_CASE("svg readiness fixtures remain parseable",
          "[readiness]") {
  const auto svg_fixtures =
      shiny::nesting::test::fixture_root().parent_path() / "files" / "tests";
  for (const auto &file_name : {"readiness_full_irregular_set.svg",
                                "readiness_partial_interlocking_set.svg",
                                "readiness_grain_exclusion_constraints.svg"}) {
    const auto polygons = parse_svg_fixture(svg_fixtures / file_name);
    CAPTURE(file_name);
    REQUIRE(polygons.size() >= 2U);
    REQUIRE(std::any_of(polygons.begin(), polygons.end(), [](const auto &entry) {
      return entry.first == "bed";
    }));
  }
}

TEST_CASE("readiness sanitizer smoke exports representative layouts",
          "[readiness-sanitizer]") {
  const auto fixtures = shiny::nesting::test::fixture_root();
  const auto svg_fixtures = fixtures.parent_path() / "files" / "tests";
  const auto test_case = ReadinessCase{
      .id = "svg-full-irregular-set",
      .request =
          request_from_svg_fixture(svg_fixtures / "readiness_full_irregular_set.svg"),
  };

  const auto artifact_dir =
      shiny::nesting::test::fixture_root().parent_path().parent_path() /
      "artifacts" / "readiness-sanitizer";
  fs::remove_all(artifact_dir);
  fs::create_directories(artifact_dir);

  const auto observation = run_readiness_case(test_case, "brkga", 17, artifact_dir);
  REQUIRE(observation.placed_parts > 0U);
}
