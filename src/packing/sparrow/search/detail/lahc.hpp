#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace shiny::nesting::search::detail {

class LateAcceptanceHistory {
public:
  LateAcceptanceHistory(const std::size_t history_length,
                        const double initial_score)
      : history_(std::max<std::size_t>(history_length, 1U), initial_score) {}

  [[nodiscard]] auto accepts(const double candidate_score,
                             const double current_score) const -> bool {
    return candidate_score >= current_score ||
           candidate_score >= history_[cursor_];
  }

  auto advance(const double score) -> void {
    history_[cursor_] = score;
    cursor_ = (cursor_ + 1U) % history_.size();
  }

private:
  std::vector<double> history_{};
  std::size_t cursor_{0};
};

} // namespace shiny::nesting::search::detail
