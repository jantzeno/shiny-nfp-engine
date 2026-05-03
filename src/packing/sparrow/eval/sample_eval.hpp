#pragma once

namespace shiny::nesting::pack::sparrow::eval {

constexpr double kLossComparisonEpsilon = 1e-9;

struct SampleEval {
  double weighted_loss{0.0};
  bool early_terminated{false};
};

struct BoundedLossContext {
  double best_known_loss{0.0};

  [[nodiscard]] auto should_stop(const double running_loss) const -> bool {
    return running_loss > best_known_loss;
  }

  [[nodiscard]] auto accepts(const SampleEval &evaluation) const -> bool {
    return !evaluation.early_terminated &&
           evaluation.weighted_loss + kLossComparisonEpsilon < best_known_loss;
  }
};

[[nodiscard]] inline auto
accumulate_weighted_loss(const BoundedLossContext &context,
                         const double running_loss, const double increment)
    -> SampleEval {
  const double next_loss = running_loss + increment;
  return {
      .weighted_loss = next_loss,
      .early_terminated = context.should_stop(next_loss),
  };
}

} // namespace shiny::nesting::pack::sparrow::eval