#include "../src/data.h"
#include "../src/toolkit.hpp"
#include "../src/waypoint.h"
#include "../src/simple_svg_1.0.0.hpp"

/*
void debugInterpolationAndTransformation() {
    svg::Document document("plot.svg",
                           svg::Layout( svg::Dimensions(6000, 8000), // was 3000 4000
                                        svg::Layout::Origin::BottomLeft,
                                        1,
                                        svg::Point(3000, 4000))); // was 200 200
    plot::plotAxes(document);

    Waypoints waypoints;
    waypoints.readFromFile();
    //waypoints.plotWaypoints(document);

    Waypoints waypoints_subset;
    waypoints.getSubset(0, 10, waypoints_subset);

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

    Waypoints inverse_transformed_waypoints;
    interpolated_waypoints.transformToMapFrame(self, inverse_transformed_waypoints);
    inverse_transformed_waypoints.plotWaypoints(document);

    document.save();
}

void interpolateWholeTrack() {
    svg::Document document("plot.svg",
                           svg::Layout( svg::Dimensions(6000, 8000), // was 3000 4000
                                        svg::Layout::Origin::BottomLeft,
                                        1,
                                        svg::Point(3000, 4000))); // was 200 200
    plot::plotAxes(document);

    Waypoints waypoints;
    waypoints.readFromFile();
    //waypoints.plotWaypoints(document);

    int subset_size = 10;
    int interpolated_size = subset_size * 10;
    for (unsigned int i = 0; i < waypoints.getWaypoints().size() - subset_size; i = i + subset_size-1) {
        Waypoints waypoints_subset;
        waypoints.getSubset(i, subset_size, waypoints_subset);

        Waypoints interpolated_waypoints;
        waypoints_subset.interpolate(interpolated_size, interpolated_waypoints);
        interpolated_waypoints.plotWaypoints(document);
    }

    document.save();
}


void generateTrajectory() {
    svg::Document document("plot.svg",
                           svg::Layout( svg::Dimensions(6000, 8000), // was 3000 4000
                                        svg::Layout::Origin::BottomLeft,
                                        1,
                                        svg::Point(3000, 4000))); // was 200 200
    plot::plotAxes(document);

    Waypoints waypoints;
    waypoints.readFromFile();
    //waypoints.plotWaypoints(document);

    Waypoints waypoints_subset;
    waypoints.getSubset(0, 10, waypoints_subset);

    Waypoints interpolated_waypoints;
    waypoints_subset.interpolate(3.0, 0, interpolated_waypoints);

    interpolated_waypoints.plotWaypoints(document);

    // TODO check for double points!

    Waypoints old_path;
    const Waypoint& wp1 = interpolated_waypoints.getWaypoints()[interpolated_waypoints.getWaypoints().size()-1];
    const Waypoint& wp2 = interpolated_waypoints.getWaypoints()[interpolated_waypoints.getWaypoints().size()-2];
    const Waypoint& wp3 = interpolated_waypoints.getWaypoints()[interpolated_waypoints.getWaypoints().size()-3];
    old_path.add(wp3);
    old_path.add(wp2);
    old_path.add(wp1);

    Car current_car;
    current_car.car_x_ = wp1.getX();
    current_car.car_y_ = wp1.getY();
    current_car.car_yaw_rad_ = atan2(wp1.getY()-wp3.getY(),wp1.getX()-wp3.getX());

    Waypoints future_waypoints;
    waypoints.getNextWaypoints(current_car, 3, future_waypoints); //current car was the end of the previous path

    Waypoints waypoints_for_interpolation;
    waypoints_for_interpolation.addAll(old_path);
    waypoints_for_interpolation.addAll(future_waypoints);

    Waypoints again_interpolated_waypoints;
    waypoints_for_interpolation.interpolate(3.0, 2, again_interpolated_waypoints); // start two points before end of last path

    again_interpolated_waypoints.plotWaypoints(document);

    document.save();
}
*/
int main() {
    //debugInterpolationAndTransformation();
    //interpolateWholeTrack();
    //generateTrajectory();
}

