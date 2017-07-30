#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "INIReader.h"
#include "simple_svg_1.0.0.hpp"

class Waypoint;
class Waypoints;
class State;
class Command;
class Car;
class Path;

class Planner {

public:

    Planner();
    ~Planner(){}

    void plan(const Waypoints &waypoints, const State &state, Command &command);

    bool checkPathSanity(const Waypoints& waypoints) const;

private:
    void generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, double requested_velocity);
    void getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const;
    void offset(const Waypoints& waypoints_to_manipulate, const Waypoints& reference_waypoints, const double& requested_d, Waypoints& offset_waypoints);

    svg::Document plot_;
    INIReader ini_reader_;

    double current_velocity_;
    double current_d_;
    double current_d_rate_;
    double current_acceleration_;
    double current_jerk_;

};

#endif
