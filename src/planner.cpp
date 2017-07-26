#include "planner.h"
#include "configuration.h"

void Planner::plan() {

    Configuration configuration("configuration/default.ini");
    configuration.read();

}
