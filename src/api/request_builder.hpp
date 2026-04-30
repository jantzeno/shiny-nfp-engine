#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "request.hpp"
#include "solve.hpp"

namespace shiny::nesting::api {

class NestingRequestBuilder {
public:
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

  auto with_execution(ExecutionPolicy execution) -> NestingRequestBuilder & {
    request_.execution = std::move(execution);
    return *this;
  }

  auto with_strategy(const StrategyKind strategy) -> NestingRequestBuilder & {
    request_.execution.strategy = strategy;
    return *this;
  }

  auto with_production_optimizer(const ProductionOptimizerKind optimizer)
      -> NestingRequestBuilder & {
    request_.execution.production_optimizer = optimizer;
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

  auto with_strategy_config(const StrategyKind strategy, const SAConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.strategy = strategy;
    request_.execution.simulated_annealing = config;
    set_primary_strategy_config(request_.execution, strategy, config);
    return *this;
  }

  auto with_strategy_config(const StrategyKind strategy,
                            const ALNSConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.strategy = strategy;
    request_.execution.alns = config;
    set_primary_strategy_config(request_.execution, strategy, config);
    return *this;
  }

  auto with_strategy_config(const StrategyKind strategy,
                            const GDRRConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.strategy = strategy;
    request_.execution.gdrr = config;
    set_primary_strategy_config(request_.execution, strategy, config);
    return *this;
  }

  auto with_strategy_config(const StrategyKind strategy,
                            const LAHCConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.strategy = strategy;
    request_.execution.lahc = config;
    set_primary_strategy_config(request_.execution, strategy, config);
    return *this;
  }

  auto with_production_config(const ProductionOptimizerKind optimizer,
                              const ProductionSearchConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.production_optimizer = optimizer;
    request_.execution.production = config;
    set_production_strategy_config(request_.execution, optimizer, config);
    return *this;
  }

  auto with_production_config(const ProductionOptimizerKind optimizer,
                              const SAConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.production_optimizer = optimizer;
    request_.execution.simulated_annealing = config;
    set_production_strategy_config(request_.execution, optimizer, config);
    return *this;
  }

  auto with_production_config(const ProductionOptimizerKind optimizer,
                              const ALNSConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.production_optimizer = optimizer;
    request_.execution.alns = config;
    set_production_strategy_config(request_.execution, optimizer, config);
    return *this;
  }

  auto with_production_config(const ProductionOptimizerKind optimizer,
                              const GDRRConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.production_optimizer = optimizer;
    request_.execution.gdrr = config;
    set_production_strategy_config(request_.execution, optimizer, config);
    return *this;
  }

  auto with_production_config(const ProductionOptimizerKind optimizer,
                              const LAHCConfig &config)
      -> NestingRequestBuilder & {
    request_.execution.production_optimizer = optimizer;
    request_.execution.lahc = config;
    set_production_strategy_config(request_.execution, optimizer, config);
    return *this;
  }

  [[nodiscard]] auto build() const -> NestingRequest { return request_; }

  [[nodiscard]] auto build_checked() const -> util::StatusOr<NestingRequest> {
    if (!request_.is_valid()) {
      return util::Status::invalid_input;
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
