#include "../src/data.h"
#include "../src/toolkit.hpp"
#include "../src/waypoint.h"
#include "../src/planner.h"
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

*/
void generateTrajectory(const Waypoints& waypoints, State& state, Planner& planner, Command& command, svg::Document& document) {
    std::cout << state.self_.s_ << std::endl;
    planner.plan(waypoints, state, command);
    //plot::plotPath(command.next_path_, document);
    plot::plotArrow(state.self_.x_, state.self_.y_, state.self_.yaw_rad_, document);
    document.save();
}


void generateTrajectoriesAroundTheTrack() {
    svg::Document document("plot.svg",
                           svg::Layout( svg::Dimensions(6000, 8000), // was 3000 4000
                                        svg::Layout::Origin::BottomLeft,
                                        1,
                                        svg::Point(3000, 4000))); // was 200 200
    plot::plotAxes(document);

    Waypoints waypoints;
    waypoints.readFromFile();

    State state_start;
    state_start.self_.x_ = 909.48;
    state_start.self_.y_ = 1128.67;
    state_start.self_.s_ = 124.8336;
    state_start.self_.d_ = 6.164833;
    state_start.self_.yaw_rad_ = 0.0;
    state_start.self_.speed_mps_ = 0.0;

    State state_towards_end;
    state_towards_end.self_.x_ = 162.952;
    state_towards_end.self_.y_ = 2313.867;
    state_towards_end.self_.s_ = 5522.24;
    state_towards_end.self_.d_ = 6.272509;
    state_towards_end.self_.yaw_rad_ = 0.0;
    state_towards_end.self_.speed_mps_ = 0.0;

    Command command;
    Planner planner;

    while (true) {
        int i = waypoints.getNextWaypointIndex(state_towards_end.self_.x_,
                                               state_towards_end.self_.y_,
                                               state_towards_end.self_.yaw_rad_);

        generateTrajectory(waypoints, state_towards_end, planner, command, document);

        state_towards_end.self_.x_ = waypoints.getWaypoints()[i].getX();
        state_towards_end.self_.y_ = waypoints.getWaypoints()[i].getY();

        double x2 = waypoints.getWaypoints()[i-1].getX();
        double y2 = waypoints.getWaypoints()[i-1].getY();

        state_towards_end.self_.yaw_rad_ = atan2(state_towards_end.self_.x_-x2, state_towards_end.self_.y_-y2);

        //std::vector<double> frenet_coordinates = conversion::toFrenet(state.self_.x_, state.self_.y_, state.self_.yaw_rad_, waypoints);

        state_towards_end.self_.s_ = waypoints.getWaypoints()[i-1].getS();
        state_towards_end.self_.d_ = 0.0;
    }
}

int main() {
    //debugInterpolationAndTransformation();
    //interpolateWholeTrack();
    generateTrajectoriesAroundTheTrack();
}

