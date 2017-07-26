#include "catch.hpp"

// class under test
#include "../src/configuration.h"

TEST_CASE( "Configuration is loaded", "[configuration]" ) {

    Configuration configuration("configuration/default.ini");
    configuration.read();

}
