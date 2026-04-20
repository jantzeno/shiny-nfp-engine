#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "packing/bounding_box/types.hpp"
#include "packing/decoder.hpp"

namespace shiny::nesting::pack {

[[nodiscard]] auto piece_metrics_for(const PieceInput &piece,
                                     std::size_t original_index)
    -> PieceOrderingMetrics;

[[nodiscard]] auto piece_order_key(const DecoderRequest &request,
                                   std::span<const std::size_t> order)
    -> std::vector<std::uint32_t>;

auto append_unique_order(const DecoderRequest &request,
                         std::vector<std::vector<std::size_t>> &orders,
                         std::vector<std::vector<std::uint32_t>> &keys,
                         std::vector<std::size_t> order) -> void;

[[nodiscard]] auto reorder_request(const DecoderRequest &request,
                                   std::span<const std::size_t> order)
    -> DecoderRequest;

[[nodiscard]] auto find_piece_index_by_id(const DecoderRequest &request,
                                          std::uint32_t piece_id)
    -> std::optional<std::size_t>;

auto move_order_index(std::vector<std::size_t> &order, std::size_t from,
                      std::size_t to) -> void;

[[nodiscard]] auto build_attempt_requests(const DecoderRequest &request)
    -> std::vector<DecoderRequest>;

} // namespace shiny::nesting::pack
