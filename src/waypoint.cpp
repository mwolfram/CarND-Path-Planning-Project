#include <vector>
#include <limits>

#include "waypoint.h"
#include "data.h"
#include "toolkit.hpp"

/**
 * Single Waypoint
 */
Waypoint::Waypoint(const double& x, const double& y, const double& s, const double& dx, const double& dy):
    x_(x),
    y_(y),
    s_(s),
    dx_(dx),
    dy_(dy)
{
}

Waypoint::Waypoint(const Waypoint &other):
    x_(other.x_),
    y_(other.y_),
    s_(other.s_),
    dx_(other.dx_),
    dy_(other.dy_)
{
}

/**
 * Multiple Waypoints
 */
Waypoints::Waypoints(const Waypoints &other):
    waypoints_(other.waypoints_),
    max_s_(other.max_s_)
{
}

void Waypoints::readFromFile() {
    // Waypoint map to read from
    string map_file = "data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    max_s_ = 6945.554;

    std::ifstream in_map(map_file.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;

        Waypoint waypoint(x, y, s, d_x, d_y);
        waypoints_.push_back(waypoint);
    }
}

void Waypoints::transformToCarFrame(const Car& self, Waypoints &transformed_waypoints) const {
    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++) {
        double wp_x = it->getX();
        double wp_y = it->getY();
        transform::toCarFrame(wp_x, wp_y, self.car_x_, self.car_y_, self.car_yaw_rad_);
        transformed_waypoints.add(Waypoint(wp_x, wp_y, it->getS(), it->getDX(), it->getY())); // TODO check if it's ok to take these values from previous waypoint or if we have to transform them too
    }
}

void Waypoints::getApproximateOriginAndDirection(double& ref_x, double& ref_y, double& ref_yaw) const {
    assert(waypoints_.size() > 1);

    int origin_index = 0;
    int target_index = 1; // TODO or use the last waypoint here?

    ref_x = waypoints_[origin_index].getX();
    ref_y = waypoints_[origin_index].getY();

    double ref_x2 = waypoints_[target_index].getX();
    double ref_y2 = waypoints_[target_index].getY();

    ref_yaw = atan2( (ref_y2-ref_y),(ref_x2-ref_x) );
}

void Waypoints::interpolate(const unsigned int& amount, Waypoints& interpolated_waypoints) {

    // determine the origin and the direction of the set of waypoints
    double ref_x, ref_y, ref_yaw;
    getApproximateOriginAndDirection(ref_x, ref_y, ref_yaw);
    Car reference_position;
    reference_position.car_x_ = ref_x;
    reference_position.car_y_ = ref_y;
    reference_position.car_yaw_rad_ = ref_yaw;

    // transform the waypoints to that frame to make sure that the X values are sorted
    Waypoints transformed_waypoints;
    transformToCarFrame(reference_position, transformed_waypoints);

    // fit a spline to the set of interpolated waypoints
    tk::spline spline;
    std::vector<double> x_coordinates;
    std::vector<double> y_coordinates;
    for (auto it = transformed_waypoints.getWaypoints().begin(); it != transformed_waypoints.getWaypoints().end(); it++) {
        x_coordinates.push_back(it->getX());
        y_coordinates.push_back(it->getY());
    }
    spline.set_points(x_coordinates, y_coordinates);

    // sample interpolated waypoints from waypoints
    double first_x = transformed_waypoints.getWaypoints()[0].getX();
    double last_x = transformed_waypoints.getWaypoints()[transformed_waypoints.getWaypoints().size()].getX();
    double range_x = last_x - first_x;
    double increment_x = range_x / amount;

    double first_s = transformed_waypoints.getWaypoints()[0].getS();
    double last_s = transformed_waypoints.getWaypoints()[transformed_waypoints.getWaypoints().size()].getS();
    double range_s = last_s - first_s;
    double increment_s = range_s / amount;

    double current_x = first_x;
    double current_s = first_s;
    for (unsigned int i = 0; i < amount; i++) {
        interpolated_waypoints.waypoints_.push_back(Waypoint(current_x, spline(current_x), current_s, 0.0, 0.0)); // TODO ok that we leave dx dy empty?
        current_x += increment_x;
        current_s += increment_s;
    }
}

void Waypoints::plotWaypoints(svg::Document& document) const {

    double pointSize = 10.0;

    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++) {

        double x = it->getX();
        double y = it->getY();

        svg::Circle circle(svg::Point(x, y),
                           pointSize,
                           svg::Fill(svg::Color(255, 0, 0)));

        document << circle;
    }
}

void Waypoints::getSubset(const unsigned int& start_index, const unsigned int &amount, Waypoints& subset) const {
    for (int i = start_index; i < start_index + amount; ++i) {
        subset.waypoints_.push_back(waypoints_[i]);
    }
}

int Waypoints::getClosestWaypointIndex(const double& x, const double& y) const {

    double minimumDistance = std::numeric_limits<double>::max();
    int closestWaypointIndex = 0;

    for(auto i = 0; i < waypoints_.size(); i++) {
        double wp_x = waypoints_[i].getX();
        double wp_y = waypoints_[i].getY();

        double distance = toolkit::distance(x, y, wp_x, wp_y);
        if(distance < minimumDistance) {
            minimumDistance = distance;
            closestWaypointIndex = i;
        }
    }

    return closestWaypointIndex;

}

int Waypoints::getNextWaypointIndex(const double& self_x, const double& self_y, const double& self_yaw_rad) const {

    int closestWaypointIndex = getClosestWaypointIndex(self_x, self_y);
    const Waypoint& closestWaypoint = waypoints_[closestWaypointIndex];

    double wp_x = closestWaypoint.getX();
    double wp_y = closestWaypoint.getY();
    double heading = atan2( (wp_y-self_y),(wp_x-self_x) );
    double angle = abs(self_yaw_rad-heading);

    if(angle > toolkit::pi()/4) {
        closestWaypointIndex++;
    }

    return closestWaypointIndex;

}

void Waypoints::getNextWaypoints(const Car& car, const unsigned int& amount, Waypoints& subset) const {
    const int next_waypoint_index = getNextWaypointIndex(car.car_x_, car.car_y_, car.car_yaw_rad_);
    getSubset(next_waypoint_index, amount, subset);
}


