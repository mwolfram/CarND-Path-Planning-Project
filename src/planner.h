#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

class Waypoint;
class State;
class Command;

class Planner {

public:

    Planner(){}
    ~Planner(){}

    void plan(const std::vector<Waypoint> &waypoints, const State &state, Command &command);

};

#endif
