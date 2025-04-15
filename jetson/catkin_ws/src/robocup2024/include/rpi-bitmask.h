//
// Created by kali on 3/14/24.
//

#include <algorithm>
#include <vector>
#include <cstdint>
#include <bitset>

#ifndef PICO_LIB_BITMASK_H
#define PICO_LIB_BITMASK_H

#define BITMASK_DEFAULT_VALUE 0

namespace RPI_Bitmask {
    enum class MOTOR_ADDRESS {
        S_SPEED = 0x00,
        NW_SPEED = 0x08,
        NE_SPEED = 0x10,
        S_DIR = 0x18,
        NW_DIR = 0x19,
        NE_DIR = 0x1A,
    };

    enum class MOTOR_SIZES {
        S_SPEED = 8,
        NW_SPEED = 8,
        NE_SPEED = 8,
        S_DIR = 1,
        NW_DIR = 1,
        NE_DIR = 1,
    };

    enum class COMPASS_ADDRESS {
        YAW = 0x1B,
    };

    enum class COMPASS_SIZES {
        YAW = 9,
    };

    enum class PERIPHERALS_ADDRESS {
        DRIBBLER = 0x24,
        KICKER = 0x2C,
    };

    enum class PERIPHERALS_SIZES {
        DRIBBLER = 8,
        KICKER = 1,
    };

    enum class LDR_ADDRESS {
        STATUS = 0x2D,
    };

    enum class LDR_SIZES {
        STATUS = 1,
    };

    enum class SETTINGS_ADDRESS {
        TEAM = 0x2E,
        ATTACK = 0x2F
    };

    enum class SETTINGS_SIZES {
        TEAM = 1,
        ATTACK = 1
    };

    struct BitmaskStructure {
        int value;
        int bitSize;
        int bitShift;
    };

    extern std::vector<BitmaskStructure> bitmask;

    uint64_t generateBitmask();
    int bitmaskToValue(int address);

    void setBitmask(uint64_t custom_bitmask);
    void setBitmaskValue(int value, int address);
}


#endif //PICO_LIB_BITMASK_H
