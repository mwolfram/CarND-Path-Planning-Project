#include "catch.hpp"

// class under test
#include "../src/waypoint.h"

TEST_CASE( "Get subset", "[get_subset]" ) {
    Waypoints waypoints;
    waypoints.readFromFile();

    Waypoints waypoints_subset;
    waypoints.getSubset(10, 10, waypoints_subset);

    REQUIRE(waypoints_subset.getWaypoints().size() == 10);
}

