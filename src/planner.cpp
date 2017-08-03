#include <iostream>
#include <algorithm>

#include "planner.h"
#include "configuration.h"
#include "toolkit.hpp"
#include "data.h"
#include "simple_svg_1.0.0.hpp"

Planner::Planner():
    internal_state_(InternalState(0.43, 0.0, 6.0))
{
}

/**
 * @brief Planner::getPoseAtEndOfPath returns position and orientation of the last point of the path
 * @param old_path
 * @param pose_at_end_of_path
 */
void Planner::getPoseAtEndOfPath(const Path& old_path, Car& pose_at_end_of_path) const {

    const unsigned int path_size = old_path.path_x_.size();
    const unsigned int path_points_to_consider_for_orientation = std::min(3u, path_size);

    assert(path_points_to_consider_for_orientation >= 2);

    const double last_position_x = old_path.path_x_[path_size-1];
    const double last_position_y = old_path.path_y_[path_size-1];

    pose_at_end_of_path.x_ = last_position_x;
    pose_at_end_of_path.y_ = last_position_y;

    const double intermediate_position_x = old_path.path_x_[path_size-path_points_to_consider_for_orientation];
    const double intermediate_position_y = old_path.path_y_[path_size-path_points_to_consider_for_orientation];

    pose_at_end_of_path.yaw_rad_ = atan2(last_position_y-intermediate_position_y, last_position_x-intermediate_position_x);
}

/**
 * @brief Planner::setD sets the d value for a set of waypoints
 * @param waypoints_to_manipulate these are the waypoints where the d value will be set
 * @param reference_waypoints these are reference waypoints for calculating d (the initial set of waypoints)
 * @param requested_d the d value that will be set
 * @param offset_waypoints the resulting waypoints, with the d value set
 */
void Planner::setD(const Waypoints& waypoints_to_manipulate, const Waypoints& reference_waypoints, const double& requested_d, Waypoints& offset_waypoints) const {

    for (auto it = waypoints_to_manipulate.getWaypoints().begin(); it != waypoints_to_manipulate.getWaypoints().end(); it++) {

        std::vector<double> xy_coordinates = conversion::toXY(it->getS(), requested_d, reference_waypoints);

        Waypoint offset_waypoint(
                    xy_coordinates[0],
                    xy_coordinates[1],
                    it->getS(),
                    it->getDX(),
                    it->getDY());

        offset_waypoints.add(offset_waypoint);
    }
}

InternalState Planner::generateTrajectory(const Waypoints& waypoints, const State& state, Command& command, const InternalState& internal_state_in) const {

    InternalState internal_state = internal_state_in;

    // configuration
    Configuration configuration;
    const double acceleration = configuration.getAcceleration();
    const double tolerated_acceleration = configuration.getToleratedAcceleration();
    const unsigned int desired_path_length = configuration.getPathLength();
    const unsigned int amount_of_future_waypoints = configuration.getAmountOfFutureWaypoints();
    const double steps_per_waypoint = configuration.getStepsPerWaypoint();
    // configuration ends here

    const unsigned int previous_path_size = state.previous_path_.path_x_.size();

    Car current_pose;

    // this could be the car itself, or something close to the end of the old path
    Waypoints existing_waypoints;

    bool use_old_path = previous_path_size > 2;
    if (use_old_path) {
        existing_waypoints.addAll(state.previous_path_);
        getPoseAtEndOfPath(state.previous_path_, current_pose);
    }
    else {
        current_pose = state.self_;
        internal_state.setCurrentVelocity(state.self_.speed_mps_ * toolkit::rate());
        existing_waypoints.add(Waypoint(current_pose.x_, current_pose.y_, current_pose.s_, 0.0, 0.0));
    }

    //
    // Add future waypoints
    //

    Waypoints future_waypoints;
    waypoints.getNextWaypoints(current_pose, amount_of_future_waypoints, future_waypoints);

    // TODO we can change this to get different behaviors for lane changes
    Waypoints future_waypoints_with_first_omitted;
    future_waypoints.getSubset(1, future_waypoints_with_first_omitted);

    Waypoints offset_future_waypoints;
    if (fabs(state.self_.d_ - internal_state.getRequestedD()) > configuration.getInLaneTolerance()) {
        // it seems like we are switching lanes, omit the first waypoint
        setD(future_waypoints_with_first_omitted, waypoints, internal_state.getRequestedD(), offset_future_waypoints);
        //std::cout << "Trajectory Generator: [ CHANGING LANE ], d: " << state.self_.d_ << ", lane_d: " << internal_state.getRequestedD() << std::endl;
    }
    else {
        // driving in lane, we can follow all waypoints
        setD(future_waypoints, waypoints, internal_state.getRequestedD(), offset_future_waypoints);
        //std::cout << "Trajectory Generator: [--> IN LANE <--], d: " << state.self_.d_ << ", lane_d: " << internal_state.getRequestedD() << std::endl;
    }

    Waypoints waypoints_for_spline;
    waypoints_for_spline.addAll(existing_waypoints);
    waypoints_for_spline.addAll(offset_future_waypoints);

    //
    // Create a spline
    //

    // determine reference pose
    Car transformation_reference_pose = waypoints_for_spline.getApproximateOriginAndDirection();

    // transform the waypoints to reference_pose frame to make sure that the X values are sorted
    Waypoints transformed_waypoints_for_spline;
    waypoints_for_spline.transformToCarFrame(transformation_reference_pose, transformed_waypoints_for_spline);

    // fit a spline to the set of transformed waypoints
    tk::spline spline;
    toolkit::fitSpline(transformed_waypoints_for_spline, spline);

    //
    // Sample from spline
    //
    // Note: the first waypoint is never sampled, neither is the last
    // For smooth paths, the last waypoint of the previous path should be used as a start
    //
    // ATTENTION: this is all happening in car frame!
    //

    const unsigned int existing_waypoints_size = existing_waypoints.getWaypoints().size();
    const Waypoint& first_waypoint_from_old_path = transformed_waypoints_for_spline.getWaypoints()[existing_waypoints_size-1];

    // determine range and step size
    double first_x = first_waypoint_from_old_path.getX();
    double first_s = first_waypoint_from_old_path.getS();
    double last_s =  transformed_waypoints_for_spline.getWaypoints()[transformed_waypoints_for_spline.getWaypoints().size()-1].getS();
    if (last_s <= first_s) {
        last_s += toolkit::maxS();
    }
    double s_since_last_wp = 0.0;
    double current_x = first_x;
    double current_y = spline(current_x);
    double current_s = first_s;

    Waypoints sampled_transformed_waypoints;

    // adjust initial velocity
    if (internal_state.getVelocityLimit() > internal_state.getCurrentVelocity()) {
        internal_state.setCurrentVelocity(internal_state.getCurrentVelocity() + acceleration);
    }
    else if (internal_state.getVelocityLimit() < internal_state.getCurrentVelocity()) {
        internal_state.setCurrentVelocity(internal_state.getCurrentVelocity() - acceleration);
    }

    double increment_x = internal_state.getCurrentVelocity() / steps_per_waypoint;

    while (sampled_transformed_waypoints.getWaypoints().size() + previous_path_size < desired_path_length) {
        double last_x = current_x;
        double last_y = current_y;
        current_x += increment_x;
        current_y = spline(current_x);
        double euclidean_distance_travelled = toolkit::distance(current_x, current_y, last_x, last_y);
        current_s += euclidean_distance_travelled;
        s_since_last_wp += euclidean_distance_travelled;

        if (current_s >= last_s) {
            break;
        }

        if (s_since_last_wp > internal_state.getCurrentVelocity()) {
            // push intermediate waypoint
            sampled_transformed_waypoints.add(Waypoint(current_x, current_y, fmod(current_s, toolkit::maxS()), 0.0, 0.0));
            s_since_last_wp = 0.0;

            // adjust velocity
            if (internal_state.getVelocityLimit() > internal_state.getCurrentVelocity()) {
                internal_state.setCurrentVelocity(internal_state.getCurrentVelocity() + acceleration);
            }
            else if (internal_state.getVelocityLimit() < internal_state.getCurrentVelocity()) {
                internal_state.setCurrentVelocity(internal_state.getCurrentVelocity() - acceleration);
            }

            increment_x = std::max(acceleration / steps_per_waypoint, internal_state.getCurrentVelocity() / steps_per_waypoint);
        }
    }

    //
    // Transform back to map frame
    //

    Waypoints sampled_waypoints;
    sampled_transformed_waypoints.transformToMapFrame(transformation_reference_pose, sampled_waypoints);


    //
    // Convert waypoints to path that can be sent as a command
    //

    Waypoints total_path_waypoints;
    total_path_waypoints.addAll(existing_waypoints);
    total_path_waypoints.addAll(sampled_waypoints);

    bool is_total_path_sane = checkPathSanity(total_path_waypoints, tolerated_acceleration);

    if (is_total_path_sane) {
        total_path_waypoints.toPath(command.next_path_);
    }
    else {
        existing_waypoints.toPath(command.next_path_);
    }

    return internal_state;
}

/**
 * @brief Planner::checkPathSanity checks that there are no significant jumps in acceleration
 * @param waypoints
 * @param tolerated_acceleration
 * @return
 */
bool Planner::checkPathSanity(const Waypoints& waypoints, double tolerated_acceleration) const {
    for (auto i = 2; i < waypoints.getWaypoints().size(); i++) {

        auto wp3 = waypoints.getWaypoints()[i-2];
        auto wp2 = waypoints.getWaypoints()[i-1];
        auto wp1 = waypoints.getWaypoints()[i];

        double distance1 = toolkit::distance(wp2.getX(), wp2.getY(), wp3.getX(), wp3.getY());
        double distance2 = toolkit::distance(wp1.getX(), wp1.getY(), wp2.getX(), wp2.getY());

        double actual_acceleration = fabs(distance2 - distance1);
        if (actual_acceleration > tolerated_acceleration) {
            std::cout << "WARNING Max Acceleration Exceeded: Actual: " << actual_acceleration << ", Tolerated: " << tolerated_acceleration << std::endl;
            return false;
        }
    }
    return true;
}

InternalState Planner::limitVelocity(State state, const InternalState& internal_state_in) const {

    InternalState internal_state = internal_state_in;

    Configuration configuration; // ATTENTION! TODO this is read multiple times!
    const double s_lookahead = configuration.getLookaheadInS();
    const double s_too_close = configuration.getTooCloseDistanceInS();
    const double lane_width = configuration.getLaneWidthToConsider();
    const double drive_slower_before_colliding_factor = configuration.getDriveSlowerBeforeCollidingFactor();

    const Car& self = state.self_;

    // Check Velocity and whether we want to change lanes
    for (auto it = state.others_.begin(); it != state.others_.end(); it++) {
        const OtherCar& other = *it;

        std::string danger = other.renderDistance(self.s_);

        if (fabs(self.d_ - other.d_) < lane_width ) {
            if (other.s_ > self.s_ && other.s_ - self.s_ < s_lookahead) {
                internal_state.setVelocityLimit(std::min(internal_state.getVelocityLimit(), toolkit::distance(0, 0, other.vx_, other.vy_) * toolkit::rate()));
            }

            if (other.s_ > self.s_ && other.s_ - self.s_ < s_too_close) {
                internal_state.setVelocityLimit(std::min(internal_state.getVelocityLimit(), toolkit::distance(0, 0, other.vx_, other.vy_) * toolkit::rate() * drive_slower_before_colliding_factor));
            }
        }
        //std::cout << danger << "      " << other.s_ << " " << other.d_ << std::endl;
    }

    //std::cout << "Limiter: Setting requested velocity to " << internal_state.getVelocityLimit() << std::endl;

    return internal_state;
}

void Planner::plan(const Waypoints& waypoints, const State& state, Command& command) {

    // configuration that is refreshed for each iteration
    Configuration configuration;
    internal_state_.setVelocityLimit(configuration.getVelocityLimit());

    // velocity limiter
    internal_state_ = limitVelocity(state, internal_state_);

    // state machine
    // TODO currently we call this second because we need the results from the limiter. If we can call the limiter offline, we can state-machine first.
    internal_state_ = state_machine_.step(state, internal_state_, configuration);

    // trajectory generator
    internal_state_ = generateTrajectory(waypoints, state, command, internal_state_);
}

