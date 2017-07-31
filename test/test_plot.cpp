#include "catch.hpp"

#include "../src/data.h"
#include "../src/toolkit.hpp"
#include "../src/waypoint.h"

// class under test
#include "../src/simple_svg_1.0.0.hpp"

TEST_CASE( "Create sample a plot", "[sample-plot]" ) {
    svg::Document document("/tmp/plot.svg");
    svg::Line line(svg::Point(20, 20), svg::Point(200,200), svg::Stroke(20, svg::Color(255,0,0)));
    document << line;
    document.save();
    // TODO assert
}

TEST_CASE( "Create a waypoint plot", "[waypoint-plot]" ) {
    svg::Document document("/tmp/plot.svg", svg::Layout(svg::Dimensions(3000, 4000)));
    Waypoints waypoints;
    waypoints.readFromFile();
    waypoints.plotWaypoints(document, svg::Color(255,0,0));
    document.save();
    // TODO assert
}
