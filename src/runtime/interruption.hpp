#pragma once

#include <functional>

namespace shiny::nesting::runtime {

/// Lightweight callable probe used to check whether an in-progress operation
/// should stop at its next safe boundary. A default-constructed probe never
/// requests interruption.
using InterruptionProbe = std::function<bool()>;

} // namespace shiny::nesting::runtime

namespace shiny::nesting::pack {

// Alias so existing code that spells pack::InterruptionProbe still compiles
// without changes.
using InterruptionProbe = runtime::InterruptionProbe;

} // namespace shiny::nesting::pack
