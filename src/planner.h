#ifndef PLANNER_H
#define PLANNER_H

class Waypoints;
class State;
class Command;

class Planner {

public:

    Planner(){}
    ~Planner(){}

    void plan(const Waypoints &waypoints, const State &state, Command &command);

};

#endif
