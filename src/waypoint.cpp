#include "waypoint.h"
#include <vector>

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

void Waypoints::readWaypoints(Waypoints &waypoints) {
    // Waypoint map to read from
    string map_file_ = "data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554; // TODO, however, this never happens

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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
        waypoints.push_back(waypoint);
    }
}

void Waypoints::transformWaypoints(std::vector<Waypoint> waypoints, const State& state) {
    for (unsigned int i = 0; i < waypoints.map_waypoints_x_.size(); i++) {
        toCarFrame(waypoints.map_waypoints_x_[i],
                   waypoints.map_waypoints_y_[i],
                   state.self_.car_x_, state.self_.car_y_, state.self_.car_yaw_rad_);
    }
}

void Waypoints::getSubsetOfWaypoints(const Waypoints& waypoints, int start_index, int amount, Waypoints& waypoints_subset) {
    for (int i = start_index; i < start_index + amount; ++i) {
        waypoints_subset.map_waypoints_x_.push_back(waypoints.map_waypoints_x_[i]);
        waypoints_subset.map_waypoints_y_.push_back(waypoints.map_waypoints_y_[i]);
        waypoints_subset.map_waypoints_s_.push_back(waypoints.map_waypoints_s_[i]);
        waypoints_subset.map_waypoints_dx_.push_back(waypoints.map_waypoints_dx_[i]);
        waypoints_subset.map_waypoints_dy_.push_back(waypoints.map_waypoints_dy_[i]);
    }
}

void Waypoints::getWaypointsPositionAndDirection(const Waypoints& waypoints, double& ref_x, double& ref_y, double& ref_yaw) {
    ref_x = waypoints.map_waypoints_x_[0];
    ref_y = waypoints.map_waypoints_y_[0];

    double ref_x2 = waypoints.map_waypoints_x_[1]; // or use the last one here?
    double ref_y2 = waypoints.map_waypoints_y_[1];

    ref_yaw = atan2( (ref_y2-ref_y),(ref_x2-ref_x) );
}

void Waypoints::interpolateWaypoints(const Waypoints& waypoints, int amount, Waypoints& interpolated_waypoints) {
    // turn the waypoints so that we get a sorted X
    double ref_x, ref_y, ref_yaw;
    getWaypointsPositionAndDirection(waypoints, ref_x, ref_y, ref_yaw);

    State state;
    state.self_.car_x_ = ref_x;
    state.self_.car_y_ = ref_y;
    state.self_.car_yaw_rad_ = ref_yaw;

    // TODO copy
    Waypoints transformed_waypoints;
    for (int i = 0; i < waypoints.map_waypoints_x_.size(); ++i) {
        transformed_waypoints.map_waypoints_x_.push_back(waypoints.map_waypoints_x_[i]);
        transformed_waypoints.map_waypoints_y_.push_back(waypoints.map_waypoints_y_[i]);
        transformed_waypoints.map_waypoints_s_.push_back(waypoints.map_waypoints_s_[i]);
        transformed_waypoints.map_waypoints_dx_.push_back(waypoints.map_waypoints_dx_[i]);
        transformed_waypoints.map_waypoints_dy_.push_back(waypoints.map_waypoints_dy_[i]);
    }

    toolkit::transformWaypoints(transformed_waypoints, state);

    tk::spline spline;
    spline.set_points(transformed_waypoints.map_waypoints_x_, transformed_waypoints.map_waypoints_y_);

    // sample interpolated waypoints from waypoints
    double xrange = transformed_waypoints.map_waypoints_x_[2] - transformed_waypoints.map_waypoints_x_[0];
    double xincrement = xrange / amount;
    double curx = transformed_waypoints.map_waypoints_x_[0];

    double srange = transformed_waypoints.map_waypoints_s_[2] - transformed_waypoints.map_waypoints_s_[0];
    double sincrement = srange / amount;
    double curs = transformed_waypoints.map_waypoints_s_[0];
    for (int i = 0; i < amount; i++) {
        interpolated_waypoints.map_waypoints_x_.push_back(curx);
        interpolated_waypoints.map_waypoints_y_.push_back(spline(curx));
        interpolated_waypoints.map_waypoints_s_.push_back(curs);
        interpolated_waypoints.map_waypoints_dx_.push_back(0.0);
        interpolated_waypoints.map_waypoints_dy_.push_back(0.0);

        curx += xincrement;
        curs += sincrement;
    }
}

void Waypoints::plotWaypoints(const Waypoints& waypoints, svg::Document& document) {

    double pointSize = 10.0;

    for (int i = 0; i < waypoints.map_waypoints_x_.size(); i++) {

        double x = waypoints.map_waypoints_x_[i];
        double y = waypoints.map_waypoints_y_[i];

        svg::Circle circle(svg::Point(x, y),
                           pointSize,
                           svg::Fill(svg::Color(255, 0, 0)));

        document << circle;
    }
}

int Waypoints::getClosestWaypoint(double x, double y, const Waypoints& waypoints) {

    const vector<double>& maps_x = waypoints.map_waypoints_x_;
    const vector<double>& maps_y = waypoints.map_waypoints_y_;

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int Waypoints::getNextWaypoint(double x, double y, double theta, const Waypoints& waypoints) {

    const vector<double>& maps_x = waypoints.map_waypoints_x_;
    const vector<double>& maps_y = waypoints.map_waypoints_y_;

    int closestWaypoint = getClosestWaypoint(x, y, waypoints);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

void Waypoint::getNextWaypoints(const Waypoints& waypoints, const State& state, const unsigned int n, Waypoints& waypoints_subset) {
    double next_x = state.self_.car_x_;
    double next_y = state.self_.car_y_;
    double next_yaw = state.self_.car_yaw_rad_;

    auto waypoints_data = waypoints.getWaypoints();
    for (auto it = waypoints_data.begin(); it != waypoints_data.end(); it++) {

        /*

        int index = Waypoints::getNextWaypoint(next_x, next_y, next_yaw, waypoints);

        waypoints_subset.map_waypoints_x_.push_back(waypoints.map_waypoints_x_[index]);
        waypoints_subset.map_waypoints_y_.push_back(waypoints.map_waypoints_y_[index]);
        waypoints_subset.map_waypoints_s_.push_back(waypoints.map_waypoints_s_[index]);
        waypoints_subset.map_waypoints_dx_.push_back(waypoints.map_waypoints_dx_[index]);
        waypoints_subset.map_waypoints_dy_.push_back(waypoints.map_waypoints_dy_[index]);

        next_x = waypoints.map_waypoints_x_[index];
        next_y = waypoints.map_waypoints_y_[index];
        */
    }
}
