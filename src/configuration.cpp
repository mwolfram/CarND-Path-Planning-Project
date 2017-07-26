#include "configuration.h"

#include <iostream>
#include "INIReader.h"

Configuration::Configuration(const std::string& configuration_file) {
    configuration_file_ = configuration_file;
}

void Configuration::read() {
    INIReader reader(configuration_file_);

    if (reader.ParseError() < 0) {
        std::cout << "Can't load 'test.ini'\n";
        return;
    }
    std::cout << reader.Get("testsection", "testentry", "notfound") << std::endl;
}

