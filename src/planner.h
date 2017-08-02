#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "configuration.h"
#include "simple_svg_1.0.0.hpp"

class Waypoint;
class Waypoints;
class State;
class Command;
class Car;
class Path;

enum Behaviour {
    KEEP_LANE,
    LANE_CHANGE
};

class Planner {

public:

    Planner();
    ~Planner(){}

    void plan(const Waypoints &waypoints, const State &state, Command &command);

    bool checkPathSanity(const Waypoints& waypoints, double tolerated_acceleration) const;
    void getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const;
    void offset(const Waypoints& waypoints_to_manipulate, const Waypoints& reference_waypoints, const double& requested_d, Waypoints& offset_waypoints) const;

private:
    void generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, double requested_velocity);
    void planBehavior(State state, double& requested_velocity);

    svg::Document plot_;

    // TODO can we get rid of this? should we?
    double current_velocity_;
    double requested_d; // TODO code style

    Behaviour behaviour_;

};

#endif
