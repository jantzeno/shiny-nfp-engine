#include "packing/irregular/constructive_packer.hpp"

#include "packing/constructive/fill_first_engine.hpp"

namespace shiny::nesting::pack {

auto IrregularConstructivePacker::solve(const NormalizedRequest &request,
                                        const SolveControl &control)
    -> std::expected<NestingResult, util::Status> {
  constructive::FillFirstEngine engine;
  return engine.solve(request, control);
}

} // namespace shiny::nesting::pack
