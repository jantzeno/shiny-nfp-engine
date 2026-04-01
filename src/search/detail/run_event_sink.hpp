#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include "search/execution.hpp"

namespace shiny::nfp::search::detail {

/**
 * @brief Serializes observer delivery and retained event history per run.
 *
 * Wraps one run-scoped observer callback plus retained event vector so
 * multiple solver threads can emit canonical search events without racing the
 * callback or retained history bookkeeping.
 *
 * @par Invariants
 * - Emission order is the lock acquisition order on this sink.
 * - Retained history stays bounded by the execution-control settings.
 *
 * @par Performance Notes
 * - Synchronization is scoped to one run and does not serialize unrelated
 *   solver runs.
 */
class RunEventSink {
public:
  RunEventSink(const SearchExecutionConfig &execution,
               std::vector<SearchEvent> &events)
      : execution_(execution), events_(events) {}

  template <class Event> auto emit(Event event) -> void {
    SearchEvent payload{std::move(event)};
    std::scoped_lock lock(mutex_);

    if (execution_.observer.installed()) {
      execution_.observer.on_event(payload);
    }

    if (execution_.control.retain_event_history &&
        execution_.control.record_event_timeline &&
        events_.size() < execution_.control.max_retained_events) {
      events_.push_back(std::move(payload));
    }
  }

private:
  const SearchExecutionConfig &execution_;
  std::vector<SearchEvent> &events_;
  std::mutex mutex_{};
};

} // namespace shiny::nfp::search::detail