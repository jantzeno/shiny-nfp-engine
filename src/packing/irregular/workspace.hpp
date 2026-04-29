#pragma once

#include "cache/nfp_cache.hpp"
#include "packing/irregular/candidate_generation.hpp"

namespace shiny::nesting::pack {

struct PackerSearchMetrics {
  std::size_t exact_nfp_cache_hits{0};
  std::size_t conservative_bbox_cache_hits{0};
  std::size_t exact_nfp_computations{0};
  std::size_t conservative_bbox_fallbacks{0};
  std::size_t conservative_bbox_candidate_points{0};
  std::size_t selected_fallback_placements{0};

  auto reset() -> void { *this = {}; }

  auto record(const CandidateGenerationDiagnostics &diagnostics) -> void {
    CandidateDiagnosticsRecorder recorder;
    recorder.record(diagnostics);
    const auto &totals = recorder.totals();
    exact_nfp_cache_hits += totals.exact_nfp_cache_hits;
    conservative_bbox_cache_hits += totals.conservative_bbox_cache_hits;
    exact_nfp_computations += totals.exact_nfp_computations;
    conservative_bbox_fallbacks += totals.conservative_bbox_fallbacks;
    conservative_bbox_candidate_points += totals.conservative_bbox_candidate_points;
  }
};

struct PackerWorkspace {
  cache::NfpCache nfp_cache{cache::default_nfp_cache_config()};
  PackerSearchMetrics search_metrics{};

  auto reset_search_metrics() -> void { search_metrics.reset(); }
};

} // namespace shiny::nesting::pack
