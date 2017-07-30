#ifndef TOOLKIT_HPP
#define TOOLKIT_HPP

#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include "waypoint.h"
#include "data.h"
#include "simple_svg_1.0.0.hpp"
#include "spline.h"

using std::vector;
using std::string;

/*
 * Toolkit
 */
namespace toolkit {

    constexpr double pi() { return M_PI; }
    constexpr double mph2mpsFactor() { return 0.44704; }

    // The max s value before wrapping around the track back to 0
    constexpr double maxS() { return 6945.554; }

    inline double deg2rad(double x) { return x * pi() / 180; }
    inline double rad2deg(double x) { return x * 180 / pi(); }

    inline double mph2mps(double x) { return x * mph2mpsFactor(); }
    inline double mps2mph(double x) { return x / mph2mpsFactor(); }

    static double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    static double normalize(const double angle_rad) {
        double TWO_PI = 2 * pi();
        return angle_rad - TWO_PI * floor((angle_rad + pi()) / TWO_PI );
    }

    static void fitSpline(const Waypoints& waypoints, tk::spline& spline) {
        std::vector<double> x_coordinates;
        std::vector<double> y_coordinates;
        for (auto it = waypoints.getWaypoints().begin(); it != waypoints.getWaypoints().end(); it++) {
            x_coordinates.push_back(it->getX());
            y_coordinates.push_back(it->getY());
        }
        spline.set_points(x_coordinates, y_coordinates);
    }

} // end of namespace toolkit


/*
 * Transform
 */
namespace transform {

    /**
     * @brief transforms a point to car coordinate system
     * @param ptx
     * @param pty
     * @param refx
     * @param refy
     * @param refpsi
     */
    // TODO rename to tf?
    static void toCarFrame(double& ptx, double& pty, const double& refx, const double& refy, const double& refpsi) {
        double deltaX = ptx - refx;
        double deltaY = pty - refy;
        double cosdiff = cos(0-refpsi);
        double sindiff = sin(0-refpsi);
        ptx = (deltaX * cosdiff - deltaY * sindiff);
        pty = (deltaX * sindiff + deltaY * cosdiff);
    }

    /**
     * @brief transforms a point from car to map frame
     * @param ptx
     * @param pty
     * @param refx
     * @param refy
     * @param refpsi
     */
    // TODO rename to inverse tf?
    static void toMapFrame(double& ptx, double& pty, const double& refx, const double& refy, const double& refpsi) {
        double inverse_refx = 0.0;
        double inverse_refy = 0.0;
        double inverse_refpsi = -refpsi;
        toCarFrame(inverse_refx, inverse_refy, refx, refy, refpsi);
        toCarFrame(ptx, pty, inverse_refx, inverse_refy, inverse_refpsi);
    }

} // end of namespace transform


/*
 * Conversion
 */
namespace conversion {

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static vector<double> toFrenet(double x, double y, double theta, const Waypoints& waypoints) {

        int next_wp = waypoints.getNextWaypointIndex(x, y, theta);

        // for the sake of legibility
        auto wp = waypoints.getWaypoints();

        int prev_wp;
        prev_wp = next_wp-1;
        if(next_wp == 0) {
            prev_wp  = wp.size()-1;
        }

        double n_x = wp[next_wp].getX()-wp[prev_wp].getX();
        double n_y = wp[next_wp].getY()-wp[prev_wp].getY();
        double x_x = x - wp[prev_wp].getX();
        double x_y = y - wp[prev_wp].getY();

        // find the projection of x onto n
        double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        double proj_x = proj_norm*n_x;
        double proj_y = proj_norm*n_y;

        double frenet_d = toolkit::distance(x_x,x_y,proj_x,proj_y);

        //see if d value is positive or negative by comparing it to a center point

        double center_x = 1000-wp[prev_wp].getX();
        double center_y = 2000-wp[prev_wp].getY();
        double centerToPos = toolkit::distance(center_x,center_y,x_x,x_y);
        double centerToRef = toolkit::distance(center_x,center_y,proj_x,proj_y);

        if(centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = 0;
        for(int i = 0; i < prev_wp; i++)
        {
            frenet_s += toolkit::distance(wp[i].getX(),wp[i].getY(),wp[i+1].getX(),wp[i+1].getY());
        }

        frenet_s += toolkit::distance(0,0,proj_x,proj_y);

        return {frenet_s,frenet_d};

    }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> toXY(double s, double d, const Waypoints& waypoints)
    {
        // for the sake of legibility
        auto wp = waypoints.getWaypoints();

        int prev_wp = -1;

        while(s > wp[prev_wp+1].getS() && (prev_wp < (int)(wp.size()-1) )) {
            prev_wp++;
        }

        int wp2 = (prev_wp+1)%wp.size();

        double heading = atan2((wp[wp2].getY()-wp[prev_wp].getY()),(wp[wp2].getX()-wp[prev_wp].getX()));
        // the x,y,s along the segment
        double seg_s = (s-wp[prev_wp].getS());

        double seg_x = wp[prev_wp].getX()+seg_s*cos(heading);
        double seg_y = wp[prev_wp].getY()+seg_s*sin(heading);

        double perp_heading = heading-toolkit::pi()/2; // TODO normalization necessary here?

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};

    }

} // end of namespace conversion


/*
 * Plot
 */
namespace plot {
    static void plotArrow(double x, double y, double direction, svg::Document& document) {

        double len = 10.0;

        double x2 = x + cos(direction) * len;
        double y2 = y + sin(direction) * len;

        svg::Line line(svg::Point(x, y), svg::Point(x2,y2), svg::Stroke(1, svg::Color(0,0,255)));
        document << line;

        double pointSize = 5.0;
        svg::Circle circle(svg::Point(x, y),
                           pointSize,
                           svg::Fill(svg::Color(0, 0, 255)));
        document << circle;
    }

    static void plotAxes(svg::Document& document) {
        double x_size = 5000.0;
        double y_size = 5000.0;
        int brightness = 160;

        svg::Line horizontal_line(svg::Point(-x_size, 0), svg::Point(x_size, 0), svg::Stroke(3, svg::Color(brightness,brightness,brightness)));
        svg::Line vertical_line(svg::Point(0, -y_size), svg::Point(0, y_size), svg::Stroke(3, svg::Color(brightness,brightness,brightness)));

        document << horizontal_line;
        document << vertical_line;
    }

    static void plotPath(const Path& path, svg::Document& document) {
        for (auto i = 0; i < path.path_x_.size(); i++) {

            double pointSize = 10.0;

            double x = path.path_x_[i];
            double y = path.path_y_[i];

            svg::Circle circle(svg::Point(x, y),
                               pointSize,
                               svg::Fill(svg::Color::Transparent),
                               svg::Stroke(1, svg::Color(0,255,0)));

            document << circle;
        }
    }

} // end of namespace plot

#endif
