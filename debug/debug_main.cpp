#include "../src/data.h"
#include "../src/toolkit.hpp"
#include "../src/simple_svg_1.0.0.hpp"

int main() {
    svg::Document document("plot.svg",
                           svg::Layout( svg::Dimensions(3000, 4000),
                                        svg::Layout::Origin::BottomLeft,
                                        1,
                                        svg::Point(200, 200)));
    plot::plotAxes(document);

    Waypoints waypoints;
    waypoints.readFromFile();

    Waypoints waypoints_subset;
    waypoints.getSubset(100, 10, waypoints_subset);

    double ref_x, ref_y, ref_yaw;
    waypoints_subset.getApproximateOriginAndDirection(ref_x, ref_y, ref_yaw);
    plot::plotArrow(ref_x, ref_y, ref_yaw, document);

    Car self;
    self.car_x_ = ref_x;
    self.car_y_ = ref_y;
    self.car_yaw_rad_ = ref_yaw;

    Waypoints transformed_waypoints;
    waypoints_subset.transformToCarFrame(self, transformed_waypoints);
    transformed_waypoints.plotWaypoints(document);

    Waypoints interpolated_waypoints;
    waypoints_subset.interpolate(100, interpolated_waypoints);
    interpolated_waypoints.plotWaypoints(document);

    document.save();
}

