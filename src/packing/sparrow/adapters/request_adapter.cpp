#include "packing/sparrow/adapters/request_adapter.hpp"

#include <algorithm>
#include <unordered_set>

namespace shiny::nesting::pack::sparrow::adapters {

namespace {

[[nodiscard]] auto find_piece_request(const NestingRequest &request,
                                      const std::uint32_t source_piece_id)
    -> const PieceRequest * {
  const auto it = std::find_if(request.pieces.begin(), request.pieces.end(),
                               [source_piece_id](const PieceRequest &piece) {
                                 return piece.piece_id == source_piece_id;
                               });
  return it == request.pieces.end() ? nullptr : &*it;
}

[[nodiscard]] auto find_bin_request(const NestingRequest &request,
                                    const std::uint32_t source_bin_id)
    -> const BinRequest * {
  const auto it = std::find_if(request.bins.begin(), request.bins.end(),
                               [source_bin_id](const BinRequest &bin) {
                                 return bin.bin_id == source_bin_id;
                               });
  return it == request.bins.end() ? nullptr : &*it;
}

} // namespace

auto to_port_instance(const NormalizedRequest &request) -> PortInstance {
  PortInstance adapted;
  adapted.selected_bin_ids = request.request.execution.selected_bin_ids;
  adapted.objective_mode = request.request.execution.objective_mode;
  adapted.allow_part_overflow = request.request.execution.allow_part_overflow;

  std::unordered_set<std::uint32_t> selected_lookup(
      adapted.selected_bin_ids.begin(), adapted.selected_bin_ids.end());

  adapted.pieces.reserve(request.expanded_pieces.size());
  adapted.assignments.reserve(request.expanded_pieces.size());
  for (std::size_t index = 0; index < request.expanded_pieces.size(); ++index) {
    const auto &expanded_piece = request.expanded_pieces[index];
    const PieceRequest *piece =
        find_piece_request(request.request, expanded_piece.source_piece_id);
    if (piece == nullptr) {
      continue;
    }

    PortPiece adapted_piece{
        .instance = expanded_piece,
        .geometry_revision = piece->geometry_revision,
        .polygon = piece->polygon,
        .value = piece->value,
        .allowed_bin_ids = piece->allowed_bin_ids,
        .pinned_bin_id = piece->assigned_bin_id,
    };
    if (index < request.forced_rotations.size()) {
      adapted_piece.forced_rotation = request.forced_rotations[index];
    }
    adapted.assignments.push_back({
        .piece_id = expanded_piece.expanded_piece_id,
        .allowed_bin_ids = adapted_piece.allowed_bin_ids,
        .pinned_bin_id = adapted_piece.pinned_bin_id,
    });
    adapted.pieces.push_back(std::move(adapted_piece));
  }

  adapted.bins.reserve(request.expanded_bins.size());
  for (const auto &expanded_bin : request.expanded_bins) {
    const BinRequest *bin =
        find_bin_request(request.request, expanded_bin.source_bin_id);
    if (bin == nullptr) {
      continue;
    }
    adapted.bins.push_back({
        .instance = expanded_bin,
        .polygon = bin->polygon,
        .start_corner = bin->start_corner,
        .exclusion_zones = bin->exclusion_zones,
        .selected = selected_lookup.empty() ||
                    selected_lookup.contains(expanded_bin.source_bin_id),
    });
  }

  adapted.frontier_bin_ids.reserve(adapted.bins.size());
  for (const auto &bin : adapted.bins) {
    if (bin.selected) {
      adapted.frontier_bin_ids.push_back(bin.instance.expanded_bin_id);
    }
  }

  return adapted;
}

auto adapt_request(const NormalizedRequest &request,
                   const SolveControl &control, const SolveProfile profile)
    -> PortRequestAdapterResult {
  return {
      .instance = to_port_instance(request),
      .seed_flow = build_seed_flow_plan(control, profile),
  };
}

auto adapt_request(const ProfileRequest &request,
                   const ProfileSolveControl &control)
    -> std::expected<PortRequestAdapterResult, util::Status> {
  const auto nesting_request_or = to_nesting_request(request);
  if (!nesting_request_or.has_value()) {
    return std::unexpected(nesting_request_or.error());
  }

  const auto normalized_request_or =
      normalize_request(nesting_request_or.value());
  if (!normalized_request_or.has_value()) {
    return std::unexpected(normalized_request_or.error());
  }

  auto adapted = PortRequestAdapterResult{
      .instance = to_port_instance(normalized_request_or.value()),
      .seed_flow = build_seed_flow_plan(control, request.profile),
  };
  adapted.instance.selected_bin_ids = request.selected_bin_ids;
  adapted.instance.objective_mode = request.objective_mode;
  adapted.instance.allow_part_overflow = request.allow_part_overflow;
  adapted.instance.maintain_bed_assignment = request.maintain_bed_assignment;
  return adapted;
}

} // namespace shiny::nesting::pack::sparrow::adapters