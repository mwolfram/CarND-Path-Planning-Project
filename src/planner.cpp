#include <iostream>
#include <algorithm>

#include "planner.h"
#include "configuration.h"
#include "toolkit.hpp"
#include "data.h"
#include "simple_svg_1.0.0.hpp"


namespace {

    int dToLane(double d) {
        double distance0 = fabs(d - 2.0);
        double distance1 = fabs(d - 6.0);
        double distance2 = fabs(d - 10.0);

        int lane = 0;
        double min_distance = 100000000.0; // TODO
        if (distance0 < min_distance) {
            lane = 0;
            min_distance = distance0;
        }
        if (distance1 < min_distance) {
            lane = 1;
            min_distance = distance1;
        }
        if (distance2 < min_distance) {
            lane = 2;
            min_distance = distance2;
        }

        return lane;
    }

    double laneToD(int lane) {
        if (lane == 0) {
            return 2.2;
        }
        else if (lane == 1) {
            return 6.0;
        }
        else if (lane == 2) {
            return 9.8;
        }
        return 50.0; // TODO not nice, but shows if there is an error
    }

}

Planner::Planner():
    plot_("plot.svg",
         svg::Layout( svg::Dimensions(6000, 8000),
                      svg::Layout::Origin::BottomLeft,
                      1,
                      svg::Point(3000, 4000))),
    ini_reader_("configuration/default.ini"),
    behaviour_(KEEP_LANE),
    requested_d(6.0)
{
    plot::plotAxes(plot_);
}

void Planner::getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const {

    const unsigned int path_size = old_path.path_x_.size();
    const unsigned int path_points_to_consider_for_orientation = std::min(3u, path_size);

    assert(path_points_to_consider_for_orientation >= 2);

    const double last_position_x = old_path.path_x_[path_size-1];
    const double last_position_y = old_path.path_y_[path_size-1];

    pose_at_end_of_path.x_ = last_position_x;
    pose_at_end_of_path.y_ = last_position_y;

    const double intermediate_position_x = old_path.path_x_[path_size-path_points_to_consider_for_orientation];
    const double intermediate_position_y = old_path.path_y_[path_size-path_points_to_consider_for_orientation];

    pose_at_end_of_path.yaw_rad_ = atan2(last_position_y-intermediate_position_y, last_position_x-intermediate_position_x);
}

void Planner::offset(const Waypoints& waypoints_to_manipulate, const Waypoints& reference_waypoints, const double& requested_d, Waypoints& offset_waypoints) const {

    // configuration
    INIReader ini("configuration/default.ini");

    // TODO start at +1 waypoint (but maybe do this somewhere else)
    for (auto it = waypoints_to_manipulate.getWaypoints().begin()+1; it != waypoints_to_manipulate.getWaypoints().end(); it++) {

        std::vector<double> xy_coordinates = conversion::toXY(it->getS(), requested_d, reference_waypoints);

        /* TODO remove
        // well, the distance should not be huge, so check this:
        double distance = toolkit::distance(xy_coordinates[0], xy_coordinates[1], it->getX(), it->getY());
        if (distance > 100.0) {
            std::cout << "exceeded distance" << std::endl;

            //repeat, so we can step through
            conversion::toXY(it->getS(), requested_d, reference_waypoints);
        }
        */

        Waypoint offset_waypoint(
                    xy_coordinates[0],
                    xy_coordinates[1],
                    it->getS(),
                    it->getDX(),
                    it->getDY());

        offset_waypoints.add(offset_waypoint);
    }
}

void Planner::generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, double requested_velocity, double requested_d) {

    // configuration
    INIReader ini("configuration/default.ini");
    const double acceleration = ini.GetReal("driving", "acceleration", 0.000001);
    const unsigned int desired_path_length = ini.GetInteger("driving", "path_length", 50);
    // TODO configuration
    const unsigned int amount_of_future_waypoints = 10u;


    const double d = requested_d;

    const unsigned int previous_path_size = state.previous_path_.path_x_.size();

    Car current_pose;

    // this could be the car itself, or something close to the end of the old path
    Waypoints existing_waypoints;

    bool use_old_path = previous_path_size > 2; // TODO this as parameter, how much to reuse
    //std::cout << "use old path: " << use_old_path << std::endl;
    if (use_old_path) {
        existing_waypoints.addAll(state.previous_path_);
        getPoseAtEndOfPath(state.previous_path_, current_pose);
    }
    else {
        current_pose = state.self_;
        current_velocity_ = state.self_.speed_mps_ * 0.02; // TODO hardcoded rate
        existing_waypoints.add(Waypoint(current_pose.x_, current_pose.y_, current_pose.s_, 0.0, 0.0));
    }

    // Now to the NEW part of the path. Note: at this point the existing_waypoints could be size 1 or larger

    //
    // Add future waypoints
    //

    // now add future waypoints, and offset them
    // current car was the end of the previous path
    Waypoints future_waypoints;
    waypoints.getNextWaypoints(current_pose, amount_of_future_waypoints, future_waypoints);

    Waypoints offset_future_waypoints;
    offset(future_waypoints, waypoints, d, offset_future_waypoints);

    Waypoints waypoints_for_spline;
    waypoints_for_spline.addAll(existing_waypoints);
    waypoints_for_spline.addAll(offset_future_waypoints);

    //
    // Create a spline
    //

    // determine reference pose
    Car transformation_reference_pose = waypoints_for_spline.getApproximateOriginAndDirection();

    // transform the waypoints to reference_pose frame to make sure that the X values are sorted
    Waypoints transformed_waypoints_for_spline;
    waypoints_for_spline.transformToCarFrame(transformation_reference_pose, transformed_waypoints_for_spline);

    // fit a spline to the set of transformed waypoints
    tk::spline spline;
    toolkit::fitSpline(transformed_waypoints_for_spline, spline);

    //
    // Sample from spline
    //
    // Note: the first waypoint is never sampled, neither is the last
    // For smooth paths, the last waypoint of the previous path should be used as a start
    //
    // ATTENTION: this is all happening in waypoints_for_spline frame!
    //

    const unsigned int existing_waypoints_size = existing_waypoints.getWaypoints().size();
    const Waypoint& first_waypoint_from_old_path = transformed_waypoints_for_spline.getWaypoints()[existing_waypoints_size-1];

    // determine range and step size
    double first_x = first_waypoint_from_old_path.getX();
    double first_s = first_waypoint_from_old_path.getS();
    double last_s =  transformed_waypoints_for_spline.getWaypoints()[transformed_waypoints_for_spline.getWaypoints().size()-1].getS();
    if (last_s <= first_s) {
        last_s += toolkit::maxS();
    }
    double s_since_last_wp = 0.0;
    double current_x = first_x;
    double current_y = spline(current_x);
    double current_s = first_s;

    //std::cout << "DEBUG RANGE AND STEP: " << std::endl;
    //std::cout << "first_s " << first_s << std::endl;
    //std::cout << "last_s " << last_s << std::endl;

    Waypoints sampled_transformed_waypoints;

    // adjust initial velocity
    if (requested_velocity > current_velocity_) {
        current_velocity_ += acceleration;
    }
    else if (requested_velocity < current_velocity_) {
        current_velocity_ -= acceleration;
    }

    double increment_x = current_velocity_ / 50.0; // TODO parameters

    int iterations = 0; // TODO debug
    while (sampled_transformed_waypoints.getWaypoints().size() + previous_path_size < desired_path_length) {

        ++iterations; // TODO debug

        double last_x = current_x;
        double last_y = current_y;
        current_x += increment_x;
        current_y = spline(current_x);
        double euclidean_distance_travelled = toolkit::distance(current_x, current_y, last_x, last_y);
        current_s += euclidean_distance_travelled;
        s_since_last_wp += euclidean_distance_travelled;

        if (current_s >= last_s) {
            break;
        }

        if (s_since_last_wp > current_velocity_) {
            // push intermediate waypoint
            sampled_transformed_waypoints.add(Waypoint(current_x, current_y, fmod(current_s, toolkit::maxS()), 0.0, 0.0)); // TODO ok that we leave dx dy empty?
            s_since_last_wp = 0.0;

            //std::cout << "waypoint distance s: " << wp_distance_s << std::endl;

            // adjust velocity
            if (requested_velocity > current_velocity_) {
                current_velocity_ += acceleration;
            }
            else if (requested_velocity < current_velocity_) {
                current_velocity_ -= acceleration;
            }

            increment_x = std::max(acceleration / 50.0, current_velocity_ / 50.0);
        }
    }

    //
    // Transform back to map frame
    //

    Waypoints sampled_waypoints;
    sampled_transformed_waypoints.transformToMapFrame(transformation_reference_pose, sampled_waypoints);


    //
    // Take a fraction of the waypoints and set it as path in the command
    //


    // TODO GET RID OF THIS , this can't be true!
    //Waypoints next_few_waypoints;
    //sampled_waypoints.getNextWaypoints(current_pose, desired_path_length-previous_path_size, next_few_waypoints);

    Waypoints total_path_waypoints;
    total_path_waypoints.addAll(existing_waypoints);
    total_path_waypoints.addAll(sampled_waypoints);

    bool is_total_path_sane = checkPathSanity(total_path_waypoints);

    if (is_total_path_sane) {
        total_path_waypoints.toPath(command.next_path_);
    }
    else {
        existing_waypoints.toPath(command.next_path_);
    }

    //std::cout << "Old Path Sanity: " << checkPathSanity(existing_waypoints) << std::endl;
    //std::cout << "Next Sanity: " << checkPathSanity(next_few_waypoints) << std::endl;
    //std::cout << "Total Sanity: " <<  << std::endl;
}

bool Planner::checkPathSanity(const Waypoints& waypoints) const {
    INIReader ini("configuration/default.ini");
    const double tolerated_acceleration = ini.GetReal("driving", "tolerated_acceleration", 0.000001);

    for (auto i = 2; i < waypoints.getWaypoints().size(); i++) {

        auto wp3 = waypoints.getWaypoints()[i-2];
        auto wp2 = waypoints.getWaypoints()[i-1];
        auto wp1 = waypoints.getWaypoints()[i];

        double distance1 = toolkit::distance(wp2.getX(), wp2.getY(), wp3.getX(), wp3.getY());
        double distance2 = toolkit::distance(wp1.getX(), wp1.getY(), wp2.getX(), wp2.getY());

        double actual_acceleration = fabs(distance2 - distance1);
        if (actual_acceleration > tolerated_acceleration) {
            std::cout << "WARNING Max Acceleration Exceeded: Actual: " << actual_acceleration << ", Tolerated: " << tolerated_acceleration << std::endl;
            return false;
        }
    }
    return true;
}

// TODO cleanup
void Planner::planBehavior(State state, double& requested_velocity, double lane_width) {

    INIReader ini("configuration/default.ini"); // ATTENTION! TODO this is read twice!
    double vel_lane_change = ini.GetReal("driving", "vel_lane_change", 0.35);
    const double s_lookahead = ini.GetReal("safety", "s_lookahead", 50.0);
    const double s_too_close = ini.GetReal("safety", "s_too_close", 30.0);


    const Car& self = state.self_;

    double drive_slower_before_colliding_factor = 0.75;

    bool try_to_switch_lanes = false;

    // Finished Lane Change
    if (abs(requested_d-self.d_) < lane_width/2.0) {
        behaviour_ = KEEP_LANE;
    }

    // Check Velocity and whether we want to change lanes
    for (auto it = state.others_.begin(); it != state.others_.end(); it++) {
        const OtherCar& other = *it;

        std::string danger = other.renderDistance(self.s_);

        if (abs(self.d_ - other.d_) < lane_width ) {
            if (other.s_ > self.s_ && other.s_ - self.s_ < s_lookahead) {
                requested_velocity = std::min(requested_velocity, toolkit::distance(0, 0, other.vx_, other.vy_) * toolkit::rate());
                if (requested_velocity < vel_lane_change && behaviour_ != LANE_CHANGE) {
                    try_to_switch_lanes = true;
                }
            }

            if (other.s_ > self.s_ && other.s_ - self.s_ < s_too_close) {
                requested_velocity = std::min(requested_velocity, toolkit::distance(0, 0, other.vx_, other.vy_) * toolkit::rate() * drive_slower_before_colliding_factor);
                if (requested_velocity < vel_lane_change  && behaviour_ != LANE_CHANGE) {
                    try_to_switch_lanes = true;
                }
            }
        }

        std::cout << danger << "      " << other.s_ << " " << other.d_ << std::endl;


    }

    if (try_to_switch_lanes) {

        int my_lane = dToLane(self.d_);

        // TODO check if the other lanes are actually free

        int target_lane = my_lane - 1;
        if (my_lane == 0) {
            target_lane = 1;
        }

        requested_d = laneToD(target_lane);
        behaviour_ = LANE_CHANGE;
        // implement the lane switch here
    }


    std::cout << "Setting requested velocity to " << requested_velocity << " and the requested d to " << requested_d << std::endl;
    std::cout << "---------------------------------" << std::endl;

    //if we are in the desired lane, which is the center of the lane +/- X,
    // --> KEEP_LANE

    //else if car in front and slow.

    // can we change left/right?
    // do we have more space forward there?



}

// TODO cleanup
void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {

    INIReader ini("configuration/default.ini"); // ATTENTION! TODO this is read twice!
    double requested_velocity = ini.GetReal("driving", "requested_velocity", 0.0);
    //double requested_d = ini.GetReal("driving", "d", 0.0);
    const double lane_width = ini.GetReal("safety", "lane_width", 4.0);

    planBehavior(state, requested_velocity, lane_width);

    generateTrajectory(waypoints, state, command, requested_velocity, requested_d);
}

