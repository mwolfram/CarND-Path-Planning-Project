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

private:
    void generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, double requested_velocity);
    void getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const;

    svg::Document plot_;
    INIReader ini_reader_;

    double current_velocity_;
    double current_acceleration_;
    double current_jerk_;

};

#endif
