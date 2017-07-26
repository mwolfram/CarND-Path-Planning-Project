#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>

class Configuration {

public:

    Configuration(const std::string& configuration_file);
    ~Configuration(){}

    void read();

private:

    std::string configuration_file_;

};

#endif
