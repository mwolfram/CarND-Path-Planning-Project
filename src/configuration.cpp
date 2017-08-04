#include "configuration.h"

#include <iostream>
#include "INIReader.h"
#include "toolkit.hpp"

Configuration::Configuration() :
    Configuration(toolkit::CONFIG_FILE)
{
}

Configuration::Configuration(const std::string &configuration_file) :
    configuration_file_(configuration_file),
    reader_(configuration_file)
{
    refresh();
}

bool Configuration::refresh() {
    reader_ = INIReader(configuration_file_);

    if (reader_.ParseError() < 0) {
        std::cout << "Configuration: Can't load " << configuration_file_ << std::endl;
        return false;
    }

    return true;
}

