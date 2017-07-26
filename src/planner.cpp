#include "planner.h"
#include "configuration.h"
#include "toolkit.hpp"
#include "data.h"

namespace {

    void goForward(const State& state, Command& command) {
        double dist_inc = 0.5;
        double car_x = state.self_.car_x_;
        double car_y = state.self_.car_y_;
        double car_yaw = state.self_.car_yaw_;

        for(int i = 0; i < 50; i++) {
              command.next_path_.path_x_.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
              command.next_path_.path_y_.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
        }
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
            angle = deg2rad(state.self_.car_yaw_);
        }
        else
        {
            pos_x = state.previous_path_.path_x_[path_size-1];
            pos_y = state.previous_path_.path_y_[path_size-1];

            double pos_x2 = state.previous_path_.path_x_[path_size-2];
            double pos_y2 = state.previous_path_.path_y_[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
        }

        double dist_inc = 0.5;
        for(int i = 0; i < 50-path_size; i++)
        {
            command.next_path_.path_x_.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
            command.next_path_.path_y_.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
            pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
        }
    }

}

void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {

    Configuration configuration("configuration/default.ini");
    configuration.read();

    //goForward(state, command);
    goInCircle(state, command);
}
