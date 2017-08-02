#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "configuration.h"

class State;
class InternalState;

enum Lane {
    LEFT, CENTER, RIGHT
};

enum Node {
    KEEP_LANE,
    CHANGE_LANE
};

class StateMachine {

public:
    StateMachine();
    InternalState step(State state, const InternalState &internal_state, Configuration configuration);

private:

    Node current_node_;
    Lane current_lane_;

    Configuration configuration_;

};

#endif
