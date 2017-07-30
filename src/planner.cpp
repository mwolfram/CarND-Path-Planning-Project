#include <iostream>

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

bool Planner::getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) {

    const unsigned int path_size = old_path.path_x_.size();
    const unsigned int path_points_to_consider_for_orientation = std::min(3, path_size);

    // we need at least two points for this to work.
    if (path_points_to_consider_for_orientation < 2) {
        return false;
    }

    double last_position_x = old_path.path_x_[path_size-1];
    double last_position_y = old_path.path_y_[path_size-1];

    pose_at_end_of_path.car_x_ = last_position_x;
    pose_at_end_of_path.car_y_ = last_position_y;

    double intermediate_position_x = old_path.path_x_[path_size-path_points_to_consider_for_orientation];
    double intermediate_position_y = old_path.path_y_[path_size-path_points_to_consider_for_orientation];

    pose_at_end_of_path.car_yaw_rad_ = atan2(last_position_y-intermediate_position_y, last_position_x-intermediate_position_x);

    return true;
}

void Planner::generateTrajectory(const Waypoints& waypoints, const State& state, Command& command) {

    // configuration
    INIReader ini("configuration/default.ini");
    const double d = ini.GetReal("driving", "d", 0.0);
    const double requested_velocity = ini.GetReal("driving", "requested_velocity", 0.0);

    Car current_pose;

    // this could be the car itself, or something close to the end of the old path
    Waypoints existing_waypoints;

    bool use_old_path = state.previous_path_.path_x_.size() > 2; // TODO this as parameter, how much to reuse
    if (use_old_path) {
        existing_waypoints.addAll(state.previous_path_);
        getPoseAtEndOfPath(state.previous_path_, current_pose);
    }
    else {
        current_pose = state.self_;
        existing_path.add(Waypoint(current_pose.car_x_, current_pose.car_y_, current_pose.car_yaw_rad_, 0.0, 0.0));
    }


    // Now to the NEW part of the path. Note: at this point the existing_waypoints could be size 1 or larger

    // we need to
    // 1. determine the continuing spline
    // 2. sample over that spline
    // --> try to get the same behaviour as before



    // now add future waypoints, and offset them
    // current car was the end of the previous path
    Waypoints future_waypoints;
    waypoints.getNextWaypoints(current_car, 3, future_waypoints);

    Waypoints offset_future_waypoints;
    future_waypoints.offset(waypoints, d, offset_future_waypoints);

    Waypoints waypoints_for_interpolation;
    if (path_size > 2) {
        waypoints_for_interpolation.addAll(previous_waypoints);
    }
    waypoints_for_interpolation.addAll(offset_future_waypoints);

    // start two points before end of last path
    Waypoints interpolated_waypoints;
    waypoints_for_interpolation.interpolate(current_velocity, target_velocity, path_size > 2 ? 2: 0, interpolated_waypoints);

    Waypoints next_few_waypoints;
    interpolated_waypoints.getNextWaypoints(current_car, 100-path_size, next_few_waypoints);

    // write these waypoints into the command
    next_few_waypoints.toPath(command.next_path_);
}

void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {
    generateTrajectory(waypoints, state, command);
}
