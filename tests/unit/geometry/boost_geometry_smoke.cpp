#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include <boost/geometry.hpp>

#include "geometry/types.hpp"

namespace bg = boost::geometry;

TEST_CASE("boost geometry headers remain available through the test build",
          "[unit][build][vendor]") {
  using Point = bg::model::d2::point_xy<double>;

  bg::model::polygon<Point> polygon;
  bg::append(polygon.outer(), Point{0.0, 0.0});
  bg::append(polygon.outer(), Point{3.0, 0.0});
  bg::append(polygon.outer(), Point{3.0, 2.0});
  bg::append(polygon.outer(), Point{0.0, 2.0});
  bg::append(polygon.outer(), Point{0.0, 0.0});
  bg::correct(polygon);

  const shiny::nesting::geom::Point2 anchor{1.0, 1.0};

  REQUIRE(anchor.x() == Catch::Approx(1.0));
  REQUIRE(bg::area(polygon) == Catch::Approx(6.0));
}

TEST_CASE("boost geometry adapts shiny geometry types directly",
          "[unit][geometry][boost]") {
  shiny::nesting::geom::PolygonWithHoles polygon{
      shiny::nesting::geom::Ring{
          {0.0, 0.0}, {3.0, 0.0}, {3.0, 2.0}, {0.0, 2.0}},
      std::vector<shiny::nesting::geom::Ring>{
          {{1.0, 0.5}, {1.0, 1.5}, {2.0, 1.5}, {2.0, 0.5}}}};

  bg::correct(polygon);

  shiny::nesting::geom::Box2 envelope{};
  bg::envelope(polygon, envelope);

  REQUIRE(bg::area(polygon) == Catch::Approx(5.0));
  REQUIRE(envelope.min.x() == Catch::Approx(0.0));
  REQUIRE(envelope.min.y() == Catch::Approx(0.0));
  REQUIRE(envelope.max.x() == Catch::Approx(3.0));
  REQUIRE(envelope.max.y() == Catch::Approx(2.0));
}