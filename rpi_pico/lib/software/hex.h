#ifndef PICO_LIB_HEX_H
#define PICO_LIB_HEX_H

#include <iostream>
#include <cstdint>
#include <string>
#include <limits>
#include <cctype> // For std::isalnum

class hex {
public:
    static uint64_t decode(std::string hex);
    static std::string encode(uint64_t value);
};


#endif //PICO_LIB_HEX_H
