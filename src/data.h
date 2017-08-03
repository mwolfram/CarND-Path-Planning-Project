#ifndef DATA_H
#define DATA_H

#include <vector>
#include <math.h>
#include <string>

class InternalState {

public:
    InternalState(const double& velocity_limit,
                  const double& current_velocity_,
                  const double& requested_d) :
        velocity_limit_(velocity_limit),
        current_velocity_(current_velocity_),
        requested_d_(requested_d)
    {
    }

    // getters
    double getVelocityLimit() const { return velocity_limit_; }
    double getCurrentVelocity() const { return current_velocity_; }
    double getRequestedD() const { return requested_d_; }

    // setters
    void setVelocityLimit(const double& velocity_limit) { velocity_limit_ = velocity_limit; }
    void setCurrentVelocity(const double& current_velocity) { current_velocity_ = current_velocity; }
    void setRequestedD(const double& requested_d) { requested_d_ = requested_d; }

private:
    double velocity_limit_;
    double current_velocity_;
    double requested_d_;

};

struct Path {

    // Path data given to the Planner
    std::vector<double> path_x_;
    std::vector<double> path_y_;

    // Path's end s and d values
    double end_path_s_;
    double end_path_d_;

};

// TODO this calls for a Pose struct

struct OtherCar {
    // Car's ID
    unsigned int id_;

    // Car's Data from Sensor Fusion
    double x_;
    double y_;
    double vx_;
    double vy_;
    double s_;
    double d_;

    OtherCar simulate(double dt) {
        // TODO the S calculation could be more exact here,
        // we are really just cutting the curve short
        OtherCar simulated_car = *this;
        simulated_car.x_ += vx_ * dt;
        simulated_car.y_ += vy_ * dt;
        simulated_car.s_ += sqrt(vx_*vx_+vy_*vy_) * dt;
        return simulated_car;
    }

    std::string renderDistance(double self_s) const {
        std::string distanceStr = "           ";
        double distance = s_ - self_s;
        if (distance > -50.0) {
            distanceStr = "#          ";
        }
        if (distance > -40.0) {
            distanceStr = " #         ";
        }
        if (distance > -30.0) {
            distanceStr = "  #        ";
        }
        if (distance > -20.0) {
            distanceStr = "   #       ";
        }
        if (distance > -10.0) {
            distanceStr = "    #      ";
        }
        if (distance > 0.0) {
            distanceStr = "     #     ";
        }
        if (distance > 10.0) {
            distanceStr = "      #    ";
        }
        if (distance > 20.0) {
            distanceStr = "       #   ";
        }
        if (distance > 30.0) {
            distanceStr = "        #  ";
        }
        if (distance > 40.0) {
            distanceStr = "         # ";
        }
        if (distance > 50.0) {
            distanceStr = "          #";
        }

        return distanceStr;
    }
};

struct Car {

    // Car's localization Data
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_rad_;
    double speed_mps_;

    Car simulate(double dt) {
        Car simulated_car = *this;
        simulated_car.x_ += speed_mps_ * dt * cos(yaw_rad_);
        simulated_car.y_ += speed_mps_ * dt * sin(yaw_rad_);
        simulated_car.s_ += speed_mps_ * dt;
        // TODO never setting d, yaw_rad, speed

        return simulated_car;
    }
};

struct State {

    Car self_;
    Path previous_path_;
    std::vector<OtherCar> others_;

    State simulate(double dt) {
        State simulated_state;

        for (auto it = others_.begin(); it != others_.end(); it++) {
            simulated_state.others_.push_back(it->simulate(dt));
        }

        simulated_state.self_ = self_.simulate(dt);
        return simulated_state;
    }

};

struct Command {

    Path next_path_;

};

#endif
