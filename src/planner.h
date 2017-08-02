#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

#include "toolkit.hpp"
#include "simple_svg_1.0.0.hpp"
#include "state_machine.h"

class Waypoint;
class Waypoints;

class Planner {

public:

    Planner();
    ~Planner(){}

    void plan(const Waypoints &waypoints, const State &state, Command &command);

    bool checkPathSanity(const Waypoints& waypoints, double tolerated_acceleration) const;
    void getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const;
    void setD(const Waypoints& waypoints_to_manipulate, const Waypoints& reference_waypoints, const double& requested_d_, Waypoints& offset_waypoints) const;

    InternalState generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, const InternalState& internal_state_in) const;
    InternalState limitVelocity(State state, const InternalState& internal_state_in) const;

private:

    InternalState internal_state_;
    StateMachine state_machine_;

};

#endif
