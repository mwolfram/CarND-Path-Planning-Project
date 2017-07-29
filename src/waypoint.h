#ifndef WAYPOINT_H
#define WAYPOINT_H

namespace svg {
    class Document;
}

class Waypoint {

public:
    Waypoint(const double& x, const double& y, const double& s, const double& dx, const double& dy);
    virtual ~Waypoint(){}
    Waypoint(const Waypoint& other);

    double getX() const {return x_;}
    double getY() const {return y_;}
    double getS() const {return s_;}
    double getDX() const {return dx_;}
    double getDY() const {return dy_;}

    static void getNextWaypoints(const Waypoints& waypoints, const State& state, const unsigned int n, Waypoints& waypoints_subset);

private:
    const double x_;
    const double y_;
    const double s_;
    const double dx_;
    const double dy_;

};

class Waypoints {

public:
    Waypoints(){}
    virtual ~Waypoints(){}
    Waypoints(const Waypoints& other);

    const std::vector<Waypoint>& getWaypoints() const {return waypoints_;}

    static void readWaypoints(Waypoints& waypoints);
    static void plotWaypoints(const Waypoints& waypoints);
    static void transformWaypoints(std::vector<Waypoint> waypoints, const State& state);
    static void getSubsetOfWaypoints(const Waypoints& waypoints, int start_index, int amount, Waypoints& waypoints_subset);
    static void getWaypointsPositionAndDirection(const Waypoints& waypoints, double& ref_x, double& ref_y, double& ref_yaw);
    static void interpolateWaypoints(const Waypoints& waypoints, int amount, Waypoints& interpolated_waypoints);
    static void plotWaypoints(const Waypoints& waypoints, svg::Document& document);
    static int getClosestWaypoint(double x, double y, const Waypoints& waypoints);
    static int getNextWaypoint(double x, double y, double theta, const Waypoints& waypoints);

private:
    std::vector<Waypoint> waypoints_;

};

#endif
