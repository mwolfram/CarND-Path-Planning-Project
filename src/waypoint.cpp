#include <vector>
#include <limits>
#include <cassert>

#include "waypoint.h"
#include "data.h"
#include "toolkit.hpp"
#include "spline.h"

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
    max_s_ = 6945.554; // TODO this has to be used when interpolating!

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
        transform::toCarFrame(wp_x, wp_y, self.x_, self.y_, self.yaw_rad_);
        transformed_waypoints.add(Waypoint(wp_x, wp_y, it->getS(), it->getDX(), it->getY())); // TODO check if it's ok to take these values from previous waypoint or if we have to transform them too
    }
}

// TODO duplicate code
void Waypoints::transformToMapFrame(const Car& self, Waypoints &transformed_waypoints) const {
    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++) {
        double wp_x = it->getX();
        double wp_y = it->getY();
        transform::toMapFrame(wp_x, wp_y, self.x_, self.y_, self.yaw_rad_);
        transformed_waypoints.add(Waypoint(wp_x, wp_y, it->getS(), it->getDX(), it->getY())); // TODO check if it's ok to take these values from previous waypoint or if we have to transform them too
    }
}

void Waypoints::addAll(const Waypoints& waypoints) {
    for (auto it = waypoints.getWaypoints().begin(); it != waypoints.getWaypoints().end(); it++) {
        waypoints_.push_back(*it);
    }
}

void Waypoints::addAll(const Path& path) {
    int path_size = path.path_x_.size();

    // fill the new path with the points from the old path
    for(int i = 0; i < path_size; i++) {
        double x = path.path_x_[i];
        double y = path.path_y_[i];
        double s = 0.0;

        // for the last path element, we know s (and even d)
        if (i == path_size-1) {
            s = path.end_path_s_;
        }

        // dx and dy are never used here
        waypoints_.push_back(Waypoint(x, y, s, 0.0, 0.0));
    }
}

void Waypoints::toPath(Path& path) const {
    for (auto it = getWaypoints().begin(); it != getWaypoints().end(); it++) {
        path.path_x_.push_back(it->getX());
        path.path_y_.push_back(it->getY());
    }
}

/**
 * @brief Waypoints::getApproximateOriginAndDirection determines the approximate origin and
 * orientation of the set of waypoints
 * @return Car estimated pose
 */
const Car Waypoints::getApproximateOriginAndDirection() const {
    assert(waypoints_.size() > 1);

    Car pose;

    int origin_index = 0;
    int target_index = waypoints_.size()-1;

    pose.x_ = waypoints_[origin_index].getX();
    pose.y_ = waypoints_[origin_index].getY();

    double ref_x2 = waypoints_[target_index].getX();
    double ref_y2 = waypoints_[target_index].getY();

    pose.yaw_rad_ = atan2( (ref_y2-pose.y_),(ref_x2-pose.x_) );

    return pose;
}

void Waypoints::offset(const Waypoints& reference_waypoints, const double& d, Waypoints& offset_waypoints) const {
    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++) {
        std::vector<double> frenet_coordinates = conversion::toFrenet(it->getX(), it->getY(), it->getDX(), reference_waypoints); // TODO calculating s would not be necessary)
        std::vector<double> xy_coordinates = conversion::toXY(frenet_coordinates[0], d, reference_waypoints);
        Waypoint offset_waypoint(
                    xy_coordinates[0],
                    xy_coordinates[1],
                    it->getS(),
                    it->getDX(),
                    it->getDY());

        offset_waypoints.add(offset_waypoint);
    }
}

void Waypoints::plotWaypoints(svg::Document& document) const {

    for (auto it = waypoints_.begin(); it != waypoints_.end(); it++) {

        float change_in_size = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/15));
        double pointSize = 3.0;// - change_in_size;

        double x = it->getX();
        double y = it->getY();

        svg::Circle circle(svg::Point(x, y),
                           pointSize,
                           svg::Fill(svg::Color::Transparent),
                           svg::Stroke(1, svg::Color(255,0,0)));

        document << circle;
    }
}

void Waypoints::getSubset(const unsigned int& start_index, const unsigned int &amount, Waypoints& subset) const {

    // we would not want to get a subset that is larger than the original set
    assert(amount <= waypoints_.size());

    for (auto i = start_index; i < start_index + amount; i = (i + 1) % waypoints_.size()) {
        subset.waypoints_.push_back(waypoints_[i]);
    }
}

void Waypoints::getSubset(const unsigned int& start_index, Waypoints& subset) const {
    for (auto i = start_index; i < waypoints_.size(); i++) {
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

void Waypoints::getRemainingWaypoints(const Car& car, Waypoints& subset) const {
    int first_waypoint_index = getNextWaypointIndex(car.x_, car.y_, car.yaw_rad_);
    getSubset(first_waypoint_index, subset);
}

void Waypoints::getNextWaypoints(const Car& car, const unsigned int& amount, Waypoints& subset, const int &amount_of_previous_to_include) const {
    int first_waypoint_index = getNextWaypointIndex(car.x_, car.y_, car.yaw_rad_);

    first_waypoint_index = (first_waypoint_index - amount_of_previous_to_include) % waypoints_.size();

    getSubset(first_waypoint_index, amount + amount_of_previous_to_include, subset);
}


