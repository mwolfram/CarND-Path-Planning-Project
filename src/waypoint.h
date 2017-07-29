#ifndef WAYPOINT_H
#define WAYPOINT_H

namespace svg {
    class Document;
}

class Car;

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

    // get
    const std::vector<Waypoint>& getWaypoints() const {return waypoints_;}

    // add
    void add(const Waypoint& waypoint) {waypoints_.push_back(waypoint);}
    void readFromFile();

    // manipulate
    void transformToCarFrame(const Car& car, Waypoints& transformed_waypoints) const;
    int getClosestWaypointIndex(const double& x, const double& y) const;
    int getNextWaypointIndex(const double& self_x, const double& self_y, const double& self_yaw_rad) const;
    void getNextWaypoints(const Car& car, const unsigned int &amount, Waypoints& subset) const;
    void getSubset(const unsigned int &start_index, const unsigned int& amount, Waypoints& subset) const;
    void getApproximateOriginAndDirection(double& ref_x, double& ref_y, double& ref_yaw) const;
    void interpolate(const unsigned int &amount, Waypoints& interpolated_waypoints);

    // plot
    void plotWaypoints(svg::Document& document) const;

private:
    std::vector<Waypoint> waypoints_;
    double max_s_;

};

#endif
