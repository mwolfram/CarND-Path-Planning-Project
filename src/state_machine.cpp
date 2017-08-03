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
            return 2.0;
        }
        else if (lane == CENTER) {
            return 6.0;
        }
        else if (lane == RIGHT) {
            return 10.0;
        }

        // unknown lane, return max double to see the error
        std::cout << "Error! Unknown lane: " << lane << std::endl;
        return std::numeric_limits<double>::max();
    }

    // feel free to forward-simulate the state before calling this
    bool isLaneFree(State state, Lane lane, Configuration configuration) {

        const double lane_free_forward = configuration.getLaneFreeForward();
        const double lane_free_backward = configuration.getLaneFreeBackward();

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

    double getFirstCarInFrontInLane(const State& state, const Lane& lane) {
        double min_distance = std::numeric_limits<double>::max();

        for (auto i = 0; i < state.others_.size(); i++) {
            const OtherCar& other = state.others_[i];

            Lane others_lane = dToLane(other.d_);
            if (others_lane == lane) {
                double offset = other.s_ - state.self_.s_;
                if (offset > 0.0 && offset < min_distance) {
                    min_distance = offset;
                }
            }
        }

        return min_distance;
    }

    bool containsLane(const Lane& lane, const std::vector<Lane>& lanes) {
        for (auto it = lanes.begin(); it != lanes.end(); it++) {
            if (lane == *it) {
                return true;
            }
        }
        return false;
    }

    Lane getMissingLane(Lane lane1, Lane lane2) {
        if (lane1 == LEFT && lane2 == CENTER) return RIGHT;
        else if (lane1 == CENTER && lane2 == RIGHT) return LEFT;
        else if (lane1 == LEFT && lane2 == RIGHT) return CENTER;
        return CENTER;
    }
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

    state = state.simulate(configuration.getLaneSwitchSimTime());

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

            double distance_in_current_lane = getFirstCarInFrontInLane(state, current_lane_);
            double max_distance = std::max(std::numeric_limits<double>::min(), distance_in_current_lane);
            Lane best_lane = current_lane_;

            for (auto it = candidates.begin(); it != candidates.end(); it++) {
                double distance_in_lane = getFirstCarInFrontInLane(state, *it);
                if (distance_in_lane > max_distance) {
                    max_distance = distance_in_lane;
                    if (fabs(distance_in_current_lane - max_distance) > configuration.getAdvantageForLaneChange()) {
                        // only if the difference between our lane and the target one is big enough
                        best_lane = *it;
                        std::cout << "Switching to " << best_lane << " because it looks better" << std::endl;
                    }
                    else if (*it == CENTER) {
                        std::cout << *it << " lane is not good enough, but I'll check the third lane" << std::endl;

                        // however if we consider changing to the center lane, let's first see the third lane
                        Lane third_lane = getMissingLane(current_lane_, *it);
                        if (isLaneFree(state, third_lane, configuration)) {
                            double distance_in_lane = getFirstCarInFrontInLane(state, *it);
                            if (fabs(distance_in_current_lane - max_distance) > configuration.getAdvantageForPreparingLaneChange() ) {
                                best_lane = *it;
                                std::cout << "Switching to " << best_lane << " because " << third_lane << " looks good." << std::endl;
                            }
                            else {
                                std::cout << third_lane << " is not good enough." << std::endl;
                            }
                        }
                    }
                }
            }
            if (best_lane != current_lane_) {
                current_node_ = CHANGE_LANE;
                current_lane_ = best_lane;
                internal_state.setRequestedD(laneToD(current_lane_));
            }
        }
        break;
    case CHANGE_LANE:
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
