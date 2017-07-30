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

// TODO this calls for a Pose struct

struct OtherCar {
    // Car's ID
    unsigned int id_;

    // Car's Data from Sensor Fusion
    double x_;
    double y_;
    double vx_;
    double vy_;
    double s_;
    double d_;
};

struct Car {

    // Car's localization Data
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_rad_;
    double speed_mps_;
};

struct State {

    Car self_;
    Path previous_path_;
    std::vector<OtherCar> others_;

};

struct Command {

    Path next_path_;

};

#endif
