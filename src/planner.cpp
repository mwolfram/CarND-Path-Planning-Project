#include <iostream>
#include <algorithm>

#include "planner.h"
#include "configuration.h"
#include "toolkit.hpp"
#include "data.h"
#include "simple_svg_1.0.0.hpp"

namespace {

    void goForward(const State& state, Command& command) {

        // configuration
        const double max_acceleration_mpss = 5;
        const double rate = 0.02;
        const double max_speed_mps = toolkit::mph2mps(49.0);


        double car_x;
        double car_y;
        double car_yaw;
        double path_point_distance = 0.0;
        double car_speed_mps = state.self_.car_speed_mps_;

        int path_size = state.previous_path_.path_x_.size();

        // fill the new path with the points from the old path
        for(int i = 0; i < path_size; i++)
        {
            command.next_path_.path_x_.push_back(state.previous_path_.path_x_[i]);
            command.next_path_.path_y_.push_back(state.previous_path_.path_y_[i]);
        }

        // analyze the path, set the car position
        if(path_size == 0)
        {
            car_x = state.self_.car_x_;
            car_y = state.self_.car_y_;
            car_yaw = state.self_.car_yaw_rad_;
        }
        else
        {
            car_x = state.previous_path_.path_x_[path_size-1];
            car_y = state.previous_path_.path_y_[path_size-1];

            double pos_x2 = state.previous_path_.path_x_[path_size-2];
            double pos_y2 = state.previous_path_.path_y_[path_size-2];

            path_point_distance = toolkit::distance(car_x, car_y, pos_x2, pos_y2);
            car_speed_mps = path_point_distance / rate;

            car_yaw = atan2(car_y-pos_y2,car_x-pos_x2);
        }


        // now to the NEW part of the path

        double next_speed_mps = car_speed_mps;
        if (next_speed_mps > max_speed_mps) {
            next_speed_mps = max_speed_mps;
        }

        std::cout << "new " << 101-path_size << std::endl;

        path_point_distance = next_speed_mps * rate;
        for(int i = 0; i < 100-path_size; i++) {

            command.next_path_.path_x_.push_back(car_x+(path_point_distance)*cos(car_yaw));
            command.next_path_.path_y_.push_back(car_y+(path_point_distance)*sin(car_yaw));

            // increase speed for next timestep
            next_speed_mps += max_acceleration_mpss * rate;
            if (next_speed_mps > max_speed_mps) {
                next_speed_mps = max_speed_mps;
            }
            path_point_distance += next_speed_mps * rate;

            //std::cout << path_point_distance << " ";
        }
        //std::cout << std::endl << std::endl;
    }

    void goInCircle(const State& state, Command& command) {

        double pos_x;
        double pos_y;
        double angle;
        int path_size = state.previous_path_.path_x_.size();

        for(int i = 0; i < path_size; i++)
        {
            command.next_path_.path_x_.push_back(state.previous_path_.path_x_[i]);
            command.next_path_.path_y_.push_back(state.previous_path_.path_y_[i]);
        }

        if(path_size == 0)
        {
            pos_x = state.self_.car_x_;
            pos_y = state.self_.car_y_;
            angle = state.self_.car_yaw_rad_;
        }
        else
        {
            pos_x = state.previous_path_.path_x_[path_size-1];
            pos_y = state.previous_path_.path_y_[path_size-1];

            double pos_x2 = state.previous_path_.path_x_[path_size-2];
            double pos_y2 = state.previous_path_.path_y_[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
        }

        double max_acceleration_mpss = 5;
        double rate = 0.02;
        double max_speed_mps = toolkit::mph2mps(50.0);
        double next_speed_mps = state.self_.car_speed_mps_;
        if (next_speed_mps > max_speed_mps) {
            next_speed_mps = max_speed_mps;
        }

        double path_point_distance = 0.0;
        for(int i = 0; i < 50-path_size; i++)
        {
            command.next_path_.path_x_.push_back(pos_x+(path_point_distance)*cos(angle+(i+1)*(toolkit::pi()/100)));
            command.next_path_.path_y_.push_back(pos_y+(path_point_distance)*sin(angle+(i+1)*(toolkit::pi()/100)));
            pos_x += (path_point_distance)*cos(angle+(i+1)*(toolkit::pi()/100));
            pos_y += (path_point_distance)*sin(angle+(i+1)*(toolkit::pi()/100));

            // increase speed for next timestep
            next_speed_mps += max_acceleration_mpss * rate;
            if (next_speed_mps > max_speed_mps) {
                next_speed_mps = max_speed_mps;
            }
            path_point_distance += next_speed_mps * rate;
        }
    }

}

Planner::Planner():
    plot_("plot.svg",
         svg::Layout( svg::Dimensions(6000, 8000),
                      svg::Layout::Origin::BottomLeft,
                      1,
                      svg::Point(3000, 4000))),
    ini_reader_("configuration/default.ini")
{
    plot::plotAxes(plot_);
    current_velocity_ = ini_reader_.GetReal("driving", "speed", 0.4); // TODO, no, this will be set by the car first
}

void Planner::getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const {

    const unsigned int path_size = old_path.path_x_.size();
    const unsigned int path_points_to_consider_for_orientation = std::min(3u, path_size);

    assert(path_points_to_consider_for_orientation >= 2);

    const double last_position_x = old_path.path_x_[path_size-1];
    const double last_position_y = old_path.path_y_[path_size-1];

    pose_at_end_of_path.car_x_ = last_position_x;
    pose_at_end_of_path.car_y_ = last_position_y;

    const double intermediate_position_x = old_path.path_x_[path_size-path_points_to_consider_for_orientation];
    const double intermediate_position_y = old_path.path_y_[path_size-path_points_to_consider_for_orientation];

    pose_at_end_of_path.car_yaw_rad_ = atan2(last_position_y-intermediate_position_y, last_position_x-intermediate_position_x);
}

void Planner::generateTrajectory(const Waypoints& waypoints, const State& state, Command& command) {

    // configuration
    INIReader ini("configuration/default.ini");
    const double d = ini.GetReal("driving", "d", 0.0);
    const double requested_velocity = ini.GetReal("driving", "requested_velocity", 0.0);

    // TODO configuration
    const unsigned int amount_of_future_waypoints = 3u;
    const unsigned int desired_path_length = 100u;

    const unsigned int previous_path_size = state.previous_path_.path_x_.size();

    Car current_pose;

    // this could be the car itself, or something close to the end of the old path
    Waypoints existing_waypoints;

    bool use_old_path = previous_path_size > 2; // TODO this as parameter, how much to reuse
    std::cout << "use old path: " << use_old_path << std::endl;
    if (use_old_path) {
        existing_waypoints.addAll(state.previous_path_);
        getPoseAtEndOfPath(state.previous_path_, current_pose);
    }
    else {
        current_pose = state.self_;
        existing_waypoints.add(Waypoint(current_pose.car_x_, current_pose.car_y_, current_pose.car_yaw_rad_, 0.0, 0.0));
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
    future_waypoints.offset(waypoints, d, offset_future_waypoints); // TODO smoothly, not like this

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
    double s_since_last_wp = 0.0;
    double increment_x = 0.01; // TODO parameters
    double current_x = first_x;
    double current_y = spline(current_x);
    double current_s = first_s;
    double acceleration = 0.01; //TODO parameters

    std::cout << "DEBUG RANGE AND STEP: " << std::endl;
    std::cout << "first_s " << first_s << std::endl;
    std::cout << "last_s " << last_s << std::endl;

    Waypoints sampled_transformed_waypoints;

    int iterations = 0; // TODO debug
    while (true) {

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
            sampled_transformed_waypoints.add(Waypoint(current_x, current_y, current_s, 0.0, 0.0)); // TODO ok that we leave dx dy empty?
            s_since_last_wp = 0.0;

            //std::cout << "waypoint distance s: " << wp_distance_s << std::endl;

            // adjust velocity
            if (requested_velocity > current_velocity_) {
                current_velocity_ += acceleration;
            }
            else if (requested_velocity < current_velocity_) {
                current_velocity_ -= acceleration;
            }
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

    Waypoints next_few_waypoints;
    sampled_waypoints.getNextWaypoints(current_pose, desired_path_length-previous_path_size, next_few_waypoints);

    // write old path into the command TODO here?
    existing_waypoints.toPath(command.next_path_);
    // alternatively, just concatenate the waypoints first.
    // or, choose a different start pose for next waypoints: the car.

    // write these waypoints into the command
    next_few_waypoints.toPath(command.next_path_);
}

void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {
    generateTrajectory(waypoints, state, command);
}

