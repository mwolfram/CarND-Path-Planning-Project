#include "catch.hpp"

// class under test
#include "../src/waypoint.h"

/*
TEST_CASE( "Interpolation of waypoints works around the track", "[waypoint_interpolation]" ) {
    Waypoints waypoints;
    waypoints.readFromFile();
    int subset_size = 10;
    int interpolated_size = 100;
    for (auto i = 0; i < 172; i++) { // TODO constant!
        Waypoints waypoints_subset;
        waypoints.getSubset(i, subset_size, waypoints_subset);

        Waypoints interpolated_waypoints;
        waypoints_subset.interpolate(interpolated_size, interpolated_waypoints);

        // require that the last waypoint is the same as the last in subset
        // same for the first
        // equidistant s-wise
        // not equidistant x-wise (dangerous assumption)

    }
}
*/

// TODO try going multiple circles (going over 0-s line)

// test back-transform


