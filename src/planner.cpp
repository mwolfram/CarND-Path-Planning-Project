#include <iostream>

#include "planner.h"
#include "configuration.h"
#include "toolkit.hpp"
#include "data.h"
#include "spline.h"

namespace {

    // TODO void stayInLaneAvoidingOtherVehicles

    void transformWaypoints(Waypoints& waypoints, const State& state) {
        for (unsigned int i = 0; i < waypoints.map_waypoints_x_.size(); i++) {
            toolkit::toCarFrame(waypoints.map_waypoints_x_[i],
                                waypoints.map_waypoints_y_[i],
                                state.self_.car_x_, state.self_.car_y_, toolkit::deg2rad(state.self_.car_yaw_));
        }
    }

    void getNextWaypoints(const Waypoints& waypoints, const State& state, const unsigned int n, Waypoints& waypoints_subset) {
        double next_x = state.self_.car_x_;
        double next_y = state.self_.car_y_;
        double next_yaw = state.self_.car_yaw_;

        for (int i = 0; i < n; i++) {
            int index = toolkit::getNextWaypoint(next_x, next_y, next_yaw, waypoints);

            waypoints_subset.map_waypoints_x_.push_back(waypoints.map_waypoints_x_[index]);
            waypoints_subset.map_waypoints_y_.push_back(waypoints.map_waypoints_y_[index]);
            waypoints_subset.map_waypoints_s_.push_back(waypoints.map_waypoints_s_[index]);
            waypoints_subset.map_waypoints_dx_.push_back(waypoints.map_waypoints_dx_[index]);
            waypoints_subset.map_waypoints_dy_.push_back(waypoints.map_waypoints_dy_[index]);

            next_x = waypoints.map_waypoints_x_[index];
            next_y = waypoints.map_waypoints_y_[index];
        }
    }

    void stayInLane(const State& state, const Waypoints& waypoints, Command& command) {

        // get the next 3 waypoints
        Waypoints waypoints_subset;
        getNextWaypoints(waypoints, state, 3, waypoints_subset);

        // transform these waypoints to car coordinates
        transformWaypoints(waypoints_subset, state);

        tk::spline spline;
        spline.set_points(waypoints_subset.map_waypoints_x_, waypoints_subset.map_waypoints_y_);
/*
        // sample from waypoints_subset;
        Waypoints sampled_waypoints;
        double xrange = waypoints_subset.map_waypoints_x_[2] - waypoints_subset.map_waypoints_x_[0];
        double xincrement = xrange / 1000.0;
        double curx = waypoints_subset.map_waypoints_x_[0];

        double srange = waypoints_subset.map_waypoints_s_[2] - waypoints_subset.map_waypoints_s_[0];
        double sincrement = srange / 1000.0;
        double curs = waypoints_subset.map_waypoints_s_[0];
        for (int i = 0; i < 1000; i++) {
            sampled_waypoints.map_waypoints_x_.push_back(curx);
            sampled_waypoints.map_waypoints_y_.push_back(spline(curx));
            sampled_waypoints.map_waypoints_s_.push_back(curs);
            sampled_waypoints.map_waypoints_dx_.push_back(0.0);
            sampled_waypoints.map_waypoints_dy_.push_back(0.0);

            curx += xincrement;
            curs += sincrement;
        }
*/
        double car_s = state.self_.car_s_;
        double car_d = state.self_.car_d_;

        double dist_inc = 0.3;
        for(int i = 0; i < 50; i++)
        {
            car_s += dist_inc;

            vector<double> xy_coordinates = toolkit::toXY(car_s, car_d, waypoints);

            command.next_path_.path_x_.push_back(xy_coordinates[0]);
            command.next_path_.path_y_.push_back(xy_coordinates[1]);
        }

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
        double car_speed_mps = toolkit::mph2mps(state.self_.car_speed_mph_);

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
            car_yaw = toolkit::deg2rad(state.self_.car_yaw_);
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

            command.next_path_.path_x_.push_back(car_x+(path_point_distance)*cos(toolkit::deg2rad(car_yaw)));
            command.next_path_.path_y_.push_back(car_y+(path_point_distance)*sin(toolkit::deg2rad(car_yaw)));

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
            angle = toolkit::deg2rad(state.self_.car_yaw_);
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
        double next_speed_mps = toolkit::mph2mps(state.self_.car_speed_mph_);;
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

void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {

    Configuration configuration("configuration/default.ini");
    //configuration.read();

    //goForward(state, command);
    //goInCircle(state, command);
    stayInLane(state, waypoints, command);
}
