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
    inline const double getVelocityLimit() { return reader_.GetReal(SECTION_DRIVING, "velocity_limit", 0.43); }
    inline const double getAcceleration() { return reader_.GetReal(SECTION_DRIVING, "acceleration", 0.0015); }
    inline const double getToleratedAcceleration() { return reader_.GetReal(SECTION_DRIVING, "tolerated_acceleration", 0.08); } // TODO deprecated?
    inline const int getPathLength() { return reader_.GetInteger(SECTION_DRIVING, "path_length", 30); }
    inline const int getAmountOfFutureWaypoints() { return reader_.GetInteger(SECTION_DRIVING, "amount_of_future_waypoints", 10); }
    inline const double getStepsPerWaypoint() { return reader_.GetReal(SECTION_DRIVING, "steps_per_waypoint", 50.0); }
    inline const double getInLaneTolerance() { return reader_.GetReal(SECTION_DRIVING, "in_lane_tolerance", 0.4); }

    // behavior
    inline const double getVelocityForLaneChange() { return reader_.GetReal(SECTION_BEHAVIOR, "vel_lane_change", 0.4); }
    inline const double getLaneFreeForward() { return reader_.GetReal(SECTION_BEHAVIOR, "lane_free_forward", 20.0); }
    inline const double getLaneFreeBackward() { return reader_.GetReal(SECTION_BEHAVIOR, "lane_free_backward", 20.0); }
    inline const double getLaneSwitchSimTime() { return reader_.GetReal(SECTION_BEHAVIOR, "lane_switch_sim_time", 5.0); }
    inline const double getAdvantageForLaneChange() { return reader_.GetReal(SECTION_BEHAVIOR, "advantage_for_lane_change", 20.0); }
    inline const double getAdvantageForPreparingLaneChange() { return reader_.GetReal(SECTION_BEHAVIOR, "advantage_for_preparing_lane_change", 40.0); }

    // safety
    inline const double getLookaheadInS() { return reader_.GetReal(SECTION_SAFETY, "s_lookahead", 30.0); }
    inline const double getTooCloseDistanceInS() { return reader_.GetReal(SECTION_SAFETY, "s_too_close", 15.0); }
    inline const double getLaneWidthToConsider() { return reader_.GetReal(SECTION_SAFETY, "lane_width", 2.0); }
    inline const double getDriveSlowerBeforeCollidingFactor() { return reader_.GetReal(SECTION_SAFETY, "drive_slower_before_colliding_factor", 0.75); }

private:

    INIReader reader_;
    const std::string configuration_file_;

};

#endif
