#include <limits>

#include "state_machine.h"
#include "toolkit.hpp"

namespace {

    Lane dToLane(double d) {
        double distance_left = fabs(d - 2.0);
        double distance_center = fabs(d - 6.0);
        double distance_right = fabs(d - 10.0);

        Lane lane = CENTER;
        double min_distance = std::numeric_limits<double>::max();
        if (distance_left < min_distance) {
            lane = LEFT;
            min_distance = distance_left;
        }
        if (distance_center < min_distance) {
            lane = CENTER;
            min_distance = distance_center;
        }
        if (distance_right < min_distance) {
            lane = RIGHT;
            min_distance = distance_right;
        }

        return lane;
    }

    double laneToD(Lane lane) {
        if (lane == LEFT) {
            return 2.2;
        }
        else if (lane == CENTER) {
            return 6.0;
        }
        else if (lane == RIGHT) {
            return 9.8;
        }

        // unknown lane, return max double to see the error
        std::cout << "Error! Unknown lane: " << lane << std::endl;
        return std::numeric_limits<double>::max();
    }

    // feel free to forward-simulate the state before calling this
    bool isLaneFree(State state, Lane lane, Configuration configuration) {

        const double lane_free_forward = configuration.getLaneFreeForward();
        const double lane_free_backward = configuration.getLaneFreeBackward();

        // Check Velocity and whether we want to change lanes
        for (auto it = state.others_.begin(); it != state.others_.end(); it++) {
            const OtherCar& other = *it;

            Lane others_lane = dToLane(other.d_);
            if (others_lane == lane) {
                double offset = other.s_ - state.self_.s_;
                if (offset > -lane_free_backward && offset < lane_free_forward) {
                    return false;
                }
            }
        }

        return true;
    }

    std::vector<Lane> getCandidates(Lane current_lane) {
        std::vector<Lane> candidates;
        switch (current_lane) {
        case LEFT:
            candidates.push_back(CENTER);
            break;
        case CENTER:
            candidates.push_back(LEFT);
            candidates.push_back(RIGHT);
            break;
        case RIGHT:
            candidates.push_back(CENTER);
            break;
        default:
            std::cout << "Error! Unknown lane: " << current_lane << std::endl;
            break;
        }

        return candidates;
    }

    //getFirstCarInFrontInLane( if false -> none, lane free!)

}

StateMachine::StateMachine() {
    current_node_ = KEEP_LANE;
    current_lane_ = CENTER;
}

InternalState StateMachine::step(State state, const InternalState& internal_state_in, Configuration configuration) {

    double vel_lane_change = configuration.getVelocityForLaneChange();
    const double lane_width = configuration.getLaneWidthToConsider();

    InternalState internal_state = internal_state_in;
    const Car& self = state.self_;

    switch(current_node_) {
    case KEEP_LANE:
        if (internal_state.getVelocityLimit() < vel_lane_change) {
            std::vector<Lane> candidates = getCandidates(current_lane_);
            std::vector<Lane> tmp_candidates;

            for (auto it = candidates.begin(); it != candidates.end(); it++) {
                if (isLaneFree(state, *it, configuration)) {
                    tmp_candidates.push_back(*it);
                }
            }

            candidates = tmp_candidates;

            if (!candidates.empty()) {
                current_node_ = CHANGE_LANE;
                current_lane_ = candidates[0];
                internal_state.setRequestedD(laneToD(current_lane_));
            }
        }

        // TODO see if we want to change lanes (close to car)
        // Yes? see which one is empty (can switch) -> candidate
        // for all candidates, choose "best"

        // Will that be
        break;
    case CHANGE_LANE:
        // TODO see if we finished changing lanes, in that case transist to KEEP_LANE
        // how do we know? the lane is already set to the target lane. if we reached that, we're done
        if (fabs(laneToD(current_lane_)-self.d_) < lane_width/2.0) {
            current_node_ = KEEP_LANE;
        }
        break;
    default:
        std::cout << "Unknown State Machine Node, defaulting to KEEP_LANE" << std::endl;
        current_node_ = KEEP_LANE;
        break;
    }

    return internal_state;

}
