#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "INIReader.h"
#include "simple_svg_1.0.0.hpp"

class Waypoint;
class Waypoints;
class State;
class Command;

class Planner {

public:

    Planner();
    ~Planner(){}

    void plan(const Waypoints &waypoints, const State &state, Command &command);

private:
    void generateTrajectory(const Waypoints& waypoints, const State& state, Command& command);

    svg::Document plot_;
    INIReader ini_reader_;

    double current_velocity_;
    double current_acceleration_;
    double current_jerk_;

};

#endif
