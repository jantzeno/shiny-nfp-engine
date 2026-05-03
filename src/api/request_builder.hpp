#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "api/solve_control.hpp"
#include "request.hpp"

namespace shiny::nesting::api {

class ProfileRequestBuilder {
public:
  auto add_bin(BinRequest bin) -> ProfileRequestBuilder & {
    request_.bins.push_back(std::move(bin));
    return *this;
  }

  auto add_piece(PieceRequest piece) -> ProfileRequestBuilder & {
    request_.pieces.push_back(std::move(piece));
    return *this;
  }

  auto with_preprocess(PreprocessPolicy preprocess) -> ProfileRequestBuilder & {
    request_.preprocess = std::move(preprocess);
    return *this;
  }

  auto with_profile(const SolveProfile profile) -> ProfileRequestBuilder & {
    request_.profile = profile;
    return *this;
  }

  auto with_objective_mode(const ObjectiveMode objective_mode)
      -> ProfileRequestBuilder & {
    request_.objective_mode = objective_mode;
    return *this;
  }

  auto with_time_limit_ms(const std::uint64_t time_limit_milliseconds)
      -> ProfileRequestBuilder & {
    request_.time_limit_milliseconds = time_limit_milliseconds;
    return *this;
  }

  auto with_selected_bins(std::vector<std::uint32_t> bin_ids)
      -> ProfileRequestBuilder & {
    request_.selected_bin_ids = std::move(bin_ids);
    return *this;
  }

  auto with_allow_part_overflow(const bool allow_part_overflow)
      -> ProfileRequestBuilder & {
    request_.allow_part_overflow = allow_part_overflow;
    return *this;
  }

  auto with_maintain_bed_assignment(const bool maintain_bed_assignment = true)
      -> ProfileRequestBuilder & {
    request_.maintain_bed_assignment = maintain_bed_assignment;
    return *this;
  }

  [[nodiscard]] auto build() const -> ProfileRequest { return request_; }

  [[nodiscard]] auto build_checked() const -> std::expected<ProfileRequest, util::Status> {
    if (!request_.is_valid()) {
      return std::unexpected(util::Status::invalid_input);
    }
    return request_;
  }

  [[nodiscard]] auto is_valid() const -> bool { return request_.is_valid(); }

private:
  ProfileRequest request_{};
};

class ProfileSolveControlBuilder {
public:
  auto with_progress(ProfileProgressObserver on_progress)
      -> ProfileSolveControlBuilder & {
    control_.on_progress = std::move(on_progress);
    return *this;
  }

  auto with_cancellation(runtime::CancellationToken cancellation)
      -> ProfileSolveControlBuilder & {
    control_.cancellation = cancellation;
    return *this;
  }

  auto with_operation_limit(const std::size_t operation_limit)
      -> ProfileSolveControlBuilder & {
    control_.operation_limit = operation_limit;
    return *this;
  }

  auto with_random_seed(const std::uint64_t random_seed)
      -> ProfileSolveControlBuilder & {
    control_.random_seed = random_seed;
    return *this;
  }

  auto with_seed_mode(const SeedProgressionMode seed_mode)
      -> ProfileSolveControlBuilder & {
    control_.seed_mode = seed_mode;
    return *this;
  }

  auto with_seed_from_rng(std::mt19937 &rng) -> ProfileSolveControlBuilder & {
    control_.random_seed = seed_from_rng(rng);
    return *this;
  }

  auto with_workspace(pack::PackerWorkspace *workspace)
      -> ProfileSolveControlBuilder & {
    control_.workspace = workspace;
    return *this;
  }

  [[nodiscard]] auto build() const -> ProfileSolveControl { return control_; }

private:
  ProfileSolveControl control_{};
};

// NestingRequestBuilder provides direct access to the bounding_box and
// metaheuristic_search (BRKGA) strategy paths with explicit execution policy
// configuration. Callers wanting profile-level control over search depth
// should prefer ProfileRequestBuilder::with_profile().
class NestingRequestBuilder {
public:
  NestingRequestBuilder() {
    request_.execution.strategy = StrategyKind::bounding_box;
    request_.execution.production_optimizer = ProductionOptimizerKind::brkga;
  }

  auto add_bin(BinRequest bin) -> NestingRequestBuilder & {
    request_.bins.push_back(std::move(bin));
    return *this;
  }

  auto add_piece(PieceRequest piece) -> NestingRequestBuilder & {
    request_.pieces.push_back(std::move(piece));
    return *this;
  }

  auto with_preprocess(PreprocessPolicy preprocess) -> NestingRequestBuilder & {
    request_.preprocess = std::move(preprocess);
    return *this;
  }

  auto with_objective_mode(const ObjectiveMode objective_mode)
      -> NestingRequestBuilder & {
    request_.execution.objective_mode = objective_mode;
    return *this;
  }

  auto with_default_rotations(geom::DiscreteRotationSet rotations)
      -> NestingRequestBuilder & {
    request_.execution.default_rotations = std::move(rotations);
    return *this;
  }

  auto with_part_spacing(const double spacing) -> NestingRequestBuilder & {
    request_.execution.part_spacing = spacing;
    return *this;
  }

  auto with_selected_bins(std::vector<std::uint32_t> bin_ids)
      -> NestingRequestBuilder & {
    request_.execution.selected_bin_ids = std::move(bin_ids);
    return *this;
  }

  auto with_irregular_options(IrregularOptions irregular)
      -> NestingRequestBuilder & {
    request_.execution.irregular = std::move(irregular);
    return *this;
  }

  auto with_bounding_box_config(pack::BoundingBoxPackingConfig config)
      -> NestingRequestBuilder & {
    request_.execution.bounding_box = std::move(config);
    return *this;
  }

  auto with_allow_part_overflow(const bool allow_part_overflow)
      -> NestingRequestBuilder & {
    request_.execution.allow_part_overflow = allow_part_overflow;
    return *this;
  }

  auto with_maintain_bed_assignment(const bool maintain_bed_assignment = true)
      -> NestingRequestBuilder & {
    request_.execution.maintain_bed_assignment = maintain_bed_assignment;
    return *this;
  }

  [[nodiscard]] auto build() const -> NestingRequest { return request_; }

  [[nodiscard]] auto build_checked() const -> std::expected<NestingRequest, util::Status> {
    if (!request_.is_valid()) {
      return std::unexpected(util::Status::invalid_input);
    }
    return request_;
  }

  [[nodiscard]] auto is_valid() const -> bool { return request_.is_valid(); }

private:
  NestingRequest request_{};
};

class SolveControlBuilder {
public:
  auto with_progress(ProgressObserver on_progress) -> SolveControlBuilder & {
    control_.on_progress = std::move(on_progress);
    return *this;
  }

  auto with_cancellation(runtime::CancellationToken cancellation)
      -> SolveControlBuilder & {
    control_.cancellation = cancellation;
    return *this;
  }

  auto with_operation_limit(const std::size_t operation_limit)
      -> SolveControlBuilder & {
    control_.operation_limit = operation_limit;
    return *this;
  }

  auto with_time_limit_ms(const std::uint64_t time_limit_milliseconds)
      -> SolveControlBuilder & {
    control_.time_limit_milliseconds = time_limit_milliseconds;
    return *this;
  }

  auto with_random_seed(const std::uint64_t random_seed)
      -> SolveControlBuilder & {
    control_.random_seed = random_seed;
    return *this;
  }

  auto with_seed_mode(const SeedProgressionMode seed_mode)
      -> SolveControlBuilder & {
    control_.seed_mode = seed_mode;
    return *this;
  }

  auto with_seed_from_rng(std::mt19937 &rng) -> SolveControlBuilder & {
    control_.random_seed = seed_from_rng(rng);
    return *this;
  }

  auto with_workspace(pack::PackerWorkspace *workspace)
      -> SolveControlBuilder & {
    control_.workspace = workspace;
    return *this;
  }

  [[nodiscard]] auto build() const -> SolveControl { return control_; }

private:
  SolveControl control_{};
};

} // namespace shiny::nesting::api
