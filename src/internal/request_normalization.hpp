#pragma once

#include "packing/bin_identity.hpp"
#include "packing/decoder.hpp"
#include "request.hpp"

namespace shiny::nesting {

struct ExpandedPieceInstance {
  std::uint32_t source_piece_id{0};
  std::uint32_t expanded_piece_id{0};
  std::uint32_t instance_index{0};
};

struct ExpandedBinInstance {
  std::uint32_t source_bin_id{0};
  std::uint32_t expanded_bin_id{0};
  std::uint32_t stock_index{0};
  pack::BinIdentity identity{};
};

struct NormalizedRequest {
  NestingRequest request{};
  std::vector<ExpandedPieceInstance> expanded_pieces{};
  std::vector<ExpandedBinInstance> expanded_bins{};
  std::vector<std::optional<geom::RotationIndex>> forced_rotations{};
};

[[nodiscard]] auto normalize_request(const NestingRequest &request)
    -> std::expected<NormalizedRequest, util::Status>;

[[nodiscard]] auto
to_bounding_box_decoder_request(const NormalizedRequest &request)
    -> std::expected<pack::DecoderRequest, util::Status>;

} // namespace shiny::nesting