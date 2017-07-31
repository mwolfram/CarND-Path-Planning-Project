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
        double car_speed_mps = state.self_.speed_mps_;

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
            car_x = state.self_.x_;
            car_y = state.self_.y_;
            car_yaw = state.self_.yaw_rad_;
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
            pos_x = state.self_.x_;
            pos_y = state.self_.y_;
            angle = state.self_.yaw_rad_;
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
        double next_speed_mps = state.self_.speed_mps_;
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

void Planner::generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, double requested_velocity) {

    // configuration
    INIReader ini("configuration/default.ini");
    const double d = ini.GetReal("driving", "d", 0.0);
    const double acceleration = ini.GetReal("driving", "acceleration", 0.000001);
    const unsigned int desired_path_length = ini.GetInteger("driving", "path_length", 50);

    // TODO configuration
    const unsigned int amount_of_future_waypoints = 10u;

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
    double increment_x = current_velocity_ / 50.0; // TODO parameters
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

            increment_x = current_velocity_ / 50.0;
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

void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {

    INIReader ini("configuration/default.ini"); // ATTENTION! TODO this is read twice!
    double requested_velocity = ini.GetReal("driving", "requested_velocity", 0.0);
    double s_lookahead = ini.GetReal("safety", "s_lookahead", 50.0);
    double s_too_close = ini.GetReal("safety", "s_too_close", 30.0);
    double lane_width = ini.GetReal("safety", "lane_width", 4.0);

    const Car& self = state.self_;

    // iterate over other cars
    for (auto it = state.others_.begin(); it != state.others_.end(); it++) {
        const OtherCar& other = *it;

        std::string danger = "                  ";

        if (abs(self.d_ - other.d_) < lane_width ) {
            // oh snap, the car is in the same lane
            if (other.s_ > self.s_ && other.s_ - self.s_ < s_lookahead) {
                // oh snap, the car is close
                danger = "Proximity Warning!";
                requested_velocity = std::min(requested_velocity, toolkit::distance(0, 0, other.vx_, other.vy_) * 0.02); //TODO hardcoded rate

            }

            if (other.s_ > self.s_ && other.s_ - self.s_ < s_too_close) {
                // oh snap, the car is very close
                danger = "Collision Warning!";
                requested_velocity = std::min(requested_velocity, toolkit::distance(0, 0, other.vx_, other.vy_) * 0.015); //TODO hardcoded rate
            }
        }

        std::cout << danger << "      " << other.s_ << " " << other.d_ << std::endl;


    }
    std::cout << "Setting requested velocity to " << requested_velocity << std::endl;
    std::cout << "---------------------------------" << std::endl;

    generateTrajectory(waypoints, state, command, requested_velocity);
}

