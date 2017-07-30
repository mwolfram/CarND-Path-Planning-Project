#include <iostream>

#include "planner.h"
#include "configuration.h"
#include "toolkit.hpp"
#include "data.h"
#include "simple_svg_1.0.0.hpp"

namespace {

    void stayInLane(const State& state, const Waypoints& waypoints, Command& command, svg::Document& document, double d) {

        INIReader ini("configuration/default.ini");
        d = ini.GetReal("driving", "offset", 0.0);

        //waypoints.plotWaypoints(document);

        int path_size = state.previous_path_.path_x_.size();
        double car_x;
        double car_y;
        double car_yaw;

        double pos_x2;
        double pos_y2;
        double pos_x3;
        double pos_y3;

        //plot::plotPath(state.previous_path_, document);

        // fill the new path with the points from the old path
        for(int i = 0; i < path_size; i++)
        {
            command.next_path_.path_x_.push_back(state.previous_path_.path_x_[i]);
            command.next_path_.path_y_.push_back(state.previous_path_.path_y_[i]);
        }

        // analyze the path, set the car position
        if(path_size < 3)
        {
            car_x = state.self_.car_x_;
            car_y = state.self_.car_y_;
            car_yaw = state.self_.car_yaw_rad_;
        }
        else
        {
            car_x = state.previous_path_.path_x_[path_size-1];
            car_y = state.previous_path_.path_y_[path_size-1];

            pos_x2 = state.previous_path_.path_x_[path_size-2];
            pos_y2 = state.previous_path_.path_y_[path_size-2];

            pos_x3 = state.previous_path_.path_x_[path_size-3];
            pos_y3 = state.previous_path_.path_y_[path_size-3];

            car_yaw = atan2(car_y-pos_y3,car_x-pos_x3);
        }

        // now to the NEW part of the path
        Car current_car;
        current_car.car_x_ = car_x;
        current_car.car_y_ = car_y;
        current_car.car_yaw_rad_ = car_yaw;

        Waypoints previous_waypoints;
        if (path_size > 2) {
            // add elements from previous path
            previous_waypoints.add(Waypoint(pos_x3, pos_y3, 0.0, 0.0, 0.0)); // TODO need S
            previous_waypoints.add(Waypoint(pos_x2, pos_y2, 0.0, 0.0, 0.0));
            previous_waypoints.add(Waypoint(car_x, car_y, state.previous_path_.end_path_s_, 0.0, 0.0));
        }

        // now add future waypoints, and offset them
        Waypoints future_waypoints;
        waypoints.getNextWaypoints(current_car, 3, future_waypoints); //current car was the end of the previous path

        Waypoints offset_future_waypoints;
        future_waypoints.offset(waypoints, d, offset_future_waypoints);

        Waypoints waypoints_for_interpolation;
        if (path_size > 2) {
            waypoints_for_interpolation.addAll(previous_waypoints);
        }
        waypoints_for_interpolation.addAll(offset_future_waypoints);

        Waypoints interpolated_waypoints;
        waypoints_for_interpolation.interpolate(0.4, path_size > 2 ? 2: 0, interpolated_waypoints); // start two points before end of last path

        //double car_s = state.self_.car_s_;
        //double car_d = state.self_.car_d_;

        //plot::plotArrow(current_car.car_x_, current_car.car_y_, current_car.car_yaw_rad_, document);

        Waypoints next_few_waypoints;
        interpolated_waypoints.getNextWaypoints(current_car, 100-path_size, next_few_waypoints);

        for (auto it = next_few_waypoints.getWaypoints().begin(); it != next_few_waypoints.getWaypoints().end(); it++) {
            command.next_path_.path_x_.push_back(it->getX());
            command.next_path_.path_y_.push_back(it->getY());
        }

        //interpolated_waypoints.plotWaypoints(document);
        //document.save();
    }

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
}



void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {
    //goForward(state, command);
    //goInCircle(state, command);
    stayInLane(state, waypoints, command, plot_, ini_reader_.GetReal("driving", "offset", 0.0));
}
