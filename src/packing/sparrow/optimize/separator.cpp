#include "packing/sparrow/optimize/separator.hpp"

#include <numeric>
#include <stop_token>

#include "packing/sparrow/eval/sample_eval.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

namespace {

[[nodiscard]] auto colliding(const quantify::CollisionTracker &tracker,
                             const std::size_t item_index) -> bool {
  if (tracker.container_loss(item_index) > 0.0) {
    return true;
  }
  for (std::size_t other = 0; other < tracker.item_count(); ++other) {
    if (other != item_index && tracker.pair_loss(item_index, other) > 0.0) {
      return true;
    }
  }
  return false;
}

} // namespace

auto run_separator(const adapters::PortPolygon &container,
                   std::span<const SeparatorItem> items,
                   const SeparatorConfig &config, runtime::SplitMix64Rng &rng,
                   runtime::TraceCapture *trace,
                   std::stop_token stoken) -> SeparatorResult {
  std::vector<quantify::CollisionTrackerItem> tracker_items;
  tracker_items.reserve(items.size());
  for (const auto &item : items) {
    tracker_items.push_back(
        {.piece_id = item.piece_id, .polygon = item.polygon});
  }
  quantify::CollisionTracker tracker(container, std::move(tracker_items));

  std::size_t strikes = 0;
  std::size_t iterations = 0;
  std::size_t accepted_moves = 0;

  while (tracker.total_loss() > 0.0 && strikes < config.strike_limit &&
         iterations < config.max_iterations &&
         !stoken.stop_requested()) {
    bool improved = false;
    std::size_t no_improvement = 0;
    std::vector<std::size_t> order(tracker.item_count());
    std::iota(order.begin(), order.end(), 0U);

    while (no_improvement < config.iter_no_improvement_limit &&
           iterations < config.max_iterations) {
      bool iteration_improved = false;
      for (const auto item_index : order) {
        if (!colliding(tracker, item_index)) {
          continue;
        }

        const auto candidate_or = sample::search_placement(
            {.tracker = &tracker,
             .moving_index = item_index,
             .allowed_rotations_degrees = config.allowed_rotations_degrees,
             .policy = config.search_policy},
            rng);
        if (!candidate_or.has_value()) {
          continue;
        }

        const auto refined = sample::refine_placement(
            tracker, item_index, candidate_or->best_candidate, config.descent);
        if (refined.best_candidate.weighted_loss +
                eval::kLossComparisonEpsilon >=
            tracker.total_loss()) {
          continue;
        }

        tracker.register_item_polygon(
            item_index,
            adapters::to_port_polygon(refined.best_candidate.polygon));
        ++accepted_moves;
        iteration_improved = true;
        if (trace != nullptr) {
          trace->accepted_moves.push_back({
              .seed = rng.seed(),
              .piece_id = tracker.item_piece_id(item_index),
              .bin_id = 0,
              .objective_before = candidate_or->best_candidate.weighted_loss,
              .objective_after = refined.best_candidate.weighted_loss,
          });
        }
      }

      tracker.update_gls_weights(config.gls_weight_cap);
      ++iterations;
      if (trace != nullptr) {
        trace->strike_counts.push_back({
            .seed = rng.seed(),
            .iteration = iterations,
            .strike_count = strikes,
        });
      }
      if (iteration_improved) {
        improved = true;
        no_improvement = 0;
      } else {
        ++no_improvement;
      }
      if (tracker.total_loss() <= 0.0) {
        break;
      }
    }

    if (!improved) {
      ++strikes;
    } else {
      strikes = 0;
    }
  }

  SeparatorResult result{
      .total_loss = tracker.total_loss(),
      .converged = tracker.total_loss() <= 1e-8,
      .iterations = iterations,
      .accepted_moves = accepted_moves,
  };
  result.polygons.reserve(tracker.item_count());
  for (std::size_t index = 0; index < tracker.item_count(); ++index) {
    result.polygons.push_back(
        adapters::to_port_polygon(tracker.item_polygon(index)));
  }
  return result;
}

} // namespace shiny::nesting::pack::sparrow::optimize