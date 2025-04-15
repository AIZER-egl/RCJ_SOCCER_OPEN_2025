//
// Created by kali on 3/31/24.
//

#include "hex.h"

std::string hex::encode(uint64_t value) {
    std::string hex = "";
    while (value > 0) {
        int remainder = value % 16;
        if (remainder < 10) {
            hex = std::to_string(remainder) + hex;
        } else {
            hex = char(remainder + 55) + hex;
        }
        value /= 16;
    }
    return hex;
}

uint64_t hex::decode(std::string hex) {
    uint64_t value = 0;

    for (char c : hex) {
        int digit;
        if (std::isalnum(c)) { // Check for alphanumeric character (0-9, A-F/a-f)
            c = std::toupper(c); // Convert to uppercase for consistent handling
            digit = c <= '9' ? c - '0' : c - 'A' + 10;
        } else {
            // Invalid character in hex string, handle error
            std::cerr << "Error: Invalid character '" << c << "' in hex string." << std::endl;
            return std::numeric_limits<uint64_t>::max(); // Or throw an exception
        }

        // Check for potential overflow before multiplication
        if (value > (std::numeric_limits<uint64_t>::max() / 16) ||
            (value == (std::numeric_limits<uint64_t>::max() / 16) && digit > (std::numeric_limits<uint64_t>::max() % 16))) {
            std::cerr << "Error: Input hex string overflows 64-bit unsigned integer." << std::endl;
            return std::numeric_limits<uint64_t>::max(); // Or throw an exception
        }

        value = value * 16 + digit;
    }

    return value;
}