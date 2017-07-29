#ifndef TOOLKIT_HPP
#define TOOLKIT_HPP

#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "data.h"

#include "spline.h"
#include "waypoint.h"
#include "simple_svg_1.0.0.hpp"

using std::vector;
using std::string;

namespace transform {

    /**
     * @brief transforms a point to car coordinate system
     * @param ptx
     * @param pty
     * @param refx
     * @param refy
     * @param refpsi
     */
    static void toCarFrame(double& ptx, double& pty, const double& refx, const double& refy, const double& refpsi) {
        double deltaX = ptx - refx;
        double deltaY = pty - refy;
        double cosdiff = cos(0-refpsi);
        double sindiff = sin(0-refpsi);
        ptx = (deltaX * cosdiff - deltaY * sindiff);
        pty = (deltaX * sindiff + deltaY * cosdiff);
    }
}

namespace toolkit {

    // For converting back and forth between radians and degrees.
    constexpr double pi() { return M_PI; }
    constexpr double mph2mpsFactor() { return 0.44704; }
    inline double deg2rad(double x) { return x * pi() / 180; }
    inline double rad2deg(double x) { return x * 180 / pi(); }
    inline double mph2mps(double x) { return x * mph2mpsFactor(); }
    inline double mps2mph(double x) { return x / mph2mpsFactor(); }

    static void plotArrow(double x, double y, double direction, svg::Document& document) {

        double len = 150.0;

        double x2 = x + cos(direction) * len;
        double y2 = y + sin(direction) * len;

        svg::Line line(svg::Point(x, y), svg::Point(x2,y2), svg::Stroke(5, svg::Color(0,0,255)));
        document << line;

        double pointSize = 30.0;
        svg::Circle circle(svg::Point(x, y),
                           pointSize,
                           svg::Fill(svg::Color(0, 0, 255)));
        document << circle;
    }

    static void plotState(const State& state) {
        // see if we can plot the state, then apply a transformation on waypoints for example

    }

    static double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }
/*
    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static vector<double> toFrenet(double x, double y, double theta, const Waypoints& waypoints) {

        const vector<double>& maps_x = waypoints.map_waypoints_x_;
        const vector<double>& maps_y = waypoints.map_waypoints_y_;

        int next_wp = getNextWaypoint(x,y, theta, waypoints);

        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0)
        {
            prev_wp  = maps_x.size()-1;
        }

        double n_x = maps_x[next_wp]-maps_x[prev_wp];
        double n_y = maps_y[next_wp]-maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];

        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;

        double frenet_d = distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        double center_x = 1000-maps_x[prev_wp];
        double center_y = 2000-maps_y[prev_wp];
        double centerToPos = distance(center_x,center_y,x_x,x_y);
        double centerToRef = distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
            frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
        }

        frenet_s += distance(0,0,proj_x,proj_y);

        return {frenet_s,frenet_d};

    }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> toXY(double s, double d, const Waypoints& waypoints) {

        const vector<double>& maps_x = waypoints.map_waypoints_x_;
        const vector<double>& maps_y = waypoints.map_waypoints_y_;
        const vector<double>& maps_s = waypoints.map_waypoints_s_;

        int prev_wp = -1;

        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
            prev_wp++;
        }

        int wp2 = (prev_wp+1)%maps_x.size();

        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

        double perp_heading = heading-pi()/2;

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};

    }
*/
} // end of namespace

#endif
