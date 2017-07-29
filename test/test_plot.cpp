#include "catch.hpp"

#include "../src/data.h"
#include "../src/toolkit.hpp"

// class under test
#include "../src/simple_svg_1.0.0.hpp"

/*
TEST_CASE( "Create sample a plot", "[sample-plot]" ) {
    svg::Document document("plot.svg");
    svg::Line line(svg::Point(20, 20), svg::Point(200,200), svg::Stroke(20, svg::Color(255,0,0)));
    document << line;
    document.save();
}
*/

/*
TEST_CASE( "Create a waypoint plot", "[waypoint-plot]" ) {
    svg::Document document("plot.svg", svg::Layout(svg::Dimensions(3000, 4000)));
    Waypoints waypoints;
    toolkit::readWaypoints(waypoints);
    toolkit::plotWaypoints(waypoints, document);
    document.save();
}
*/

TEST_CASE( "Create a waypoint plot", "[waypoint-plot]" ) {
    svg::Document document("plot.svg", svg::Layout(svg::Dimensions(3000, 4000)));
    Waypoints waypoints;
    Waypoints waypoints_subset;
    toolkit::readWaypoints(waypoints);
    toolkit::getSubsetOfWaypoints(waypoints, 100, 10, waypoints_subset);

    // see if we can infer position and orientation
    double ref_x, ref_y, ref_yaw;
    toolkit::getWaypointsPositionAndDirection(waypoints_subset, ref_x, ref_y, ref_yaw);
    toolkit::plotArrow(ref_x, ref_y, ref_yaw, document);

    State state;
    state.self_.car_x_ = ref_x;
    state.self_.car_y_ = ref_y;
    state.self_.car_yaw_rad_ = ref_yaw;

    Waypoints interpolated_waypoints;
    toolkit::interpolateWaypoints(waypoints_subset, 100, interpolated_waypoints);
    toolkit::plotWaypoints(interpolated_waypoints, document);

    document.save();
}
