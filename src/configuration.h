#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include "INIReader.h"

namespace {
    static const std::string SECTION_DRIVING = "driving";
    static const std::string SECTION_BEHAVIOR = "behavior";
    static const std::string SECTION_SAFETY = "safety";
}

class Configuration {

public:

    Configuration();
    Configuration(const std::string& configuration_file);
    ~Configuration(){}

    void refresh();

    // driving
    inline const double getRequestedVelocity() { return reader_.GetReal(SECTION_DRIVING, "requested_velocity", 0.43); }
    inline const double getAcceleration() { return reader_.GetReal(SECTION_DRIVING, "acceleration", 0.0015); }
    inline const double getToleratedAcceleration() { return reader_.GetReal(SECTION_DRIVING, "tolerated_acceleration", 0.08); } // TODO deprecated
    inline const int getPathLength() { return reader_.GetInteger(SECTION_DRIVING, "path_length", 30); }
    inline const int getAmountOfFutureWaypoints() { return reader_.GetInteger(SECTION_DRIVING, "amount_of_future_waypoints", 10); }
    inline const double getStepsPerWaypoint() { return reader_.GetReal(SECTION_DRIVING, "steps_per_waypoint", 50.0); }

    // behavior
    inline const double getVelocityForLaneChange() { return reader_.GetReal(SECTION_BEHAVIOR, "vel_lane_change", 0.4); }

    // safety
    inline const double getLookaheadInS() { return reader_.GetReal(SECTION_SAFETY, "s_lookahead", 30.0); }
    inline const double getTooCloseDistanceInS() { return reader_.GetReal(SECTION_SAFETY, "s_too_close", 15.0); }
    inline const double getLaneWidthToConsider() { return reader_.GetReal(SECTION_SAFETY, "lane_width", 2.0); }

private:

    INIReader reader_;
    const std::string configuration_file_;

};

#endif
