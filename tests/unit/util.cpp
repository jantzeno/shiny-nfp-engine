#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "util/logging.hpp"
#include "util/status.hpp"

TEST_CASE("status or carries values and error states", "[util][status]") {
  shiny::nfp::util::StatusOr<std::vector<int>> value_or(
      std::vector<int>{1, 2, 3});

  REQUIRE(value_or.ok());
  REQUIRE(static_cast<bool>(value_or));
  REQUIRE(value_or.status() == shiny::nfp::util::Status::ok);
  REQUIRE(value_or.value().size() == 3U);

  shiny::nfp::util::StatusOr<std::string> error_or(
      shiny::nfp::util::Status::invalid_input);
  REQUIRE_FALSE(error_or.ok());
  REQUIRE_FALSE(static_cast<bool>(error_or));
  REQUIRE(error_or.status() == shiny::nfp::util::Status::invalid_input);

  SHINY_NFP_LOG(info, "status-or test exercised logging hook");
}