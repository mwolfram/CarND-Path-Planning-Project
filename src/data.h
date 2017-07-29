#ifndef DATA_H
#define DATA_H

#include <vector>

struct Path {

    // Path data given to the Planner
    std::vector<double> path_x_;
    std::vector<double> path_y_;

    // Path's end s and d values
    double end_path_s_;
    double end_path_d_;

};

struct Car {

    // Car's ID
    unsigned int id_;

    // Car's localization Data
    double car_x_;
    double car_y_;
    double car_s_;
    double car_d_;
    double car_yaw_rad_;
    double car_speed_mps_;
};

struct State {

    Car self_;
    Path previous_path_;
    std::vector<Car> others_;

};

struct Command {

    Path next_path_;

};

#endif
