#include <iostream>

#include "../../tools/support/sample_cases.hpp"
#include "search/masonry.hpp"

namespace {

void print_event(const shiny::nfp::search::SearchEvent &event) {
  std::cout << "observer_event="
            << shiny::nfp::search::to_string(
                   shiny::nfp::search::search_event_kind(event))
            << '\n';
}

} // namespace

auto main() -> int {
  shiny::nfp::search::MasonryRunner runner;
  auto request = shiny::nfp::tooling::make_masonry_run_request();
  request.execution.observer.on_event =
      [&](const shiny::nfp::search::SearchEvent &event) { print_event(event); };

  const auto result = runner.run(request);

  std::cout << "algorithm=" << shiny::nfp::to_string(result.algorithm) << '\n';
  std::cout << "bins_used=" << result.masonry.bins.size() << '\n';
  std::cout << "unplaced_pieces="
            << result.masonry.layout.unplaced_piece_ids.size() << '\n';
  std::cout << "masonry_progress_steps=" << result.progress.size() << '\n';
  std::cout << "when_to_prefer=masonry is best for deterministic shelf or "
               "row-oriented layouts; use constructive_decoder when greedy "
               "contact-boundary placement is the primary objective"
            << '\n';

  for (const auto &entry : result.masonry.trace) {
    std::cout << "piece=" << entry.piece_id << " bin=" << entry.bin_id
              << " shelf=" << entry.shelf_index << " translation=("
              << entry.translation.x << ',' << entry.translation.y << ')';
    if (entry.started_new_shelf) {
      std::cout << " [new_shelf]";
    }
    if (entry.opened_new_bin) {
      std::cout << " [new_bin]";
    }
    std::cout << '\n';
  }

  return 0;
}