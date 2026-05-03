#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "util/logging.hpp"
#include "util/status.hpp"

TEST_CASE("Expected carries values and error states", "[util][status]") {
  std::expected<std::vector<int>, shiny::nesting::util::Status> value_or(
      std::vector<int>{1, 2, 3});

  REQUIRE(value_or.has_value());
  REQUIRE(static_cast<bool>(value_or));
  REQUIRE(value_or.value().size() == 3U);

  std::expected<std::string, shiny::nesting::util::Status> error_or(
      std::unexpected(shiny::nesting::util::Status::invalid_input));
  REQUIRE_FALSE(error_or.has_value());
  REQUIRE_FALSE(static_cast<bool>(error_or));
  REQUIRE(error_or.error() == shiny::nesting::util::Status::invalid_input);

  SHINY_NESTING_LOG(info, "status-or test exercised logging hook");
}
