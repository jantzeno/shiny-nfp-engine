#pragma once

#include <expected>
#include <optional>
#include <string_view>
#include <utility>

namespace shiny::nesting::util {

/**
 * @brief Minimal status code used by IO and utility helpers.
 */
enum class Status {
  ok = 0,
  invalid_input = 1,
  computation_failed = 2,
  cache_miss = 3,
  not_implemented = 4,
};

[[nodiscard]] constexpr auto status_name(const Status status)
    -> std::string_view {
  switch (status) {
  case Status::ok:
    return "ok";
  case Status::invalid_input:
    return "invalid_input";
  case Status::computation_failed:
    return "computation_failed";
  case Status::cache_miss:
    return "cache_miss";
  case Status::not_implemented:
    return "not_implemented";
  }
  return "unknown";
}

/**
 * @brief Minimal status-or-value transport for non-exception APIs.
 *
 * @par Invariants
 * - A successful instance holds both `Status::ok` and a value.
 */
template <class T> class StatusOr {
public:
  /**
   * @brief Constructs a successful result from a copied value.
   */
  StatusOr(const T &value) : status_(Status::ok), value_(value) {}

  /**
   * @brief Constructs a successful result from a moved value.
   */
  StatusOr(T &&value) : status_(Status::ok), value_(std::move(value)) {}

  /**
   * @brief Constructs a failing result with no value.
   */
  StatusOr(Status status) : status_(status) {}

  /**
   * @brief Reports whether the instance contains a successful value.
   */
  [[nodiscard]] auto ok() const -> bool {
    return status_ == Status::ok && value_.has_value();
  }

  /**
   * @brief Reports whether the instance contains a successful value.
   */
  [[nodiscard]] explicit operator bool() const { return ok(); }

  /**
   * @brief Returns the current status code.
   */
  [[nodiscard]] auto status() const -> Status {
    return ok() ? Status::ok : status_;
  }

  /**
   * @brief Reports whether a value payload is present.
   */
  [[nodiscard]] auto has_value() const -> bool { return value_.has_value(); }

  /**
   * @brief Returns the stored value by lvalue reference.
   */
  [[nodiscard]] auto value() & -> T & { return *value_; }

  /**
   * @brief Returns the stored value by const lvalue reference.
   */
  [[nodiscard]] auto value() const & -> const T & { return *value_; }

  /**
   * @brief Returns the stored value by rvalue reference.
   */
  [[nodiscard]] auto value() && -> T && { return std::move(*value_); }

private:
  Status status_{Status::ok};
  std::optional<T> value_{};
};

/**
 * @brief Preferred return type for new public APIs and Sparrow-port boundaries.
 *
 * Type alias for `std::expected<T, Status>`. Use this for all new internal
 * Sparrow-port functions and new public solve helpers. Keep `StatusOr<T>` on
 * existing surfaces until they are materially rewritten.
 *
 * @see to_expected() for bridging old-style StatusOr returns into new code.
 */
template <class T> using Expected = std::expected<T, Status>;

/**
 * @brief Bridges a `StatusOr<T>` into the `Expected<T>` vocabulary.
 *
 * Use at integration points where an old-style return meets a new Sparrow-port
 * boundary that expects `util::Expected<T>`.
 */
template <class T>
[[nodiscard]] auto to_expected(StatusOr<T> &&source) -> Expected<T> {
  if (source.ok()) {
    return std::move(source).value();
  }
  return std::unexpected(source.status());
}

} // namespace shiny::nesting::util