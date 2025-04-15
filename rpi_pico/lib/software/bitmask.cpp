//
// Created by kali on 3/14/24.
//

#include <iostream>
#include "bitmask.h"

namespace Bitmask {
    std::vector<BitmaskStructure> bitmask = {
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::S_SPEED),
                static_cast<int>(MOTOR_ADDRESS::S_SPEED)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::NW_SPEED),
                static_cast<int>(MOTOR_ADDRESS::NW_SPEED)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::NE_SPEED),
                static_cast<int>(MOTOR_ADDRESS::NE_SPEED)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::S_DIR),
                static_cast<int>(MOTOR_ADDRESS::S_DIR)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::NW_DIR),
                static_cast<int>(MOTOR_ADDRESS::NW_DIR)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::NE_DIR),
                static_cast<int>(MOTOR_ADDRESS::NE_DIR)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(COMPASS_SIZES::YAW),
                static_cast<int>(COMPASS_ADDRESS::YAW)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(PERIPHERALS_SIZES::DRIBBLER),
                static_cast<int>(PERIPHERALS_ADDRESS::DRIBBLER)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(PERIPHERALS_SIZES::KICKER),
                static_cast<int>(PERIPHERALS_ADDRESS::KICKER)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::STATUS),
                static_cast<int>(LDR_ADDRESS::STATUS)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(SETTINGS_SIZES::TEAM),
                static_cast<int>(SETTINGS_ADDRESS::TEAM)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(SETTINGS_SIZES::ATTACK),
                static_cast<int>(SETTINGS_ADDRESS::ATTACK),
            }
    };

    uint64_t generateBitmask () {
        uint64_t generatedBitmask = 0;
        for (int i = 0; i < bitmask.size(); i++) {
            generatedBitmask |= (static_cast<uint64_t>(bitmask[i].value) << bitmask[i].bitShift);
        }
        return generatedBitmask;
    }

    int bitmaskToValue(int address) {

        auto element = std::find_if(bitmask.begin(), bitmask.end(), [address](const BitmaskStructure &element) {
            return element.bitShift == address;
        });

        if (element -> bitShift == address) {
            return element -> value;
        }

        return -1;
    }

    void setBitmaskValue(int value, int address) {
        auto element = std::find_if(bitmask.begin(), bitmask.end(), [address](const BitmaskStructure &element) {
            return element.bitShift == address;
        });

        if (element -> bitShift == address) {
            element -> value = value;
        }
    }

    void setBitmask(uint64_t custom_bitmask) {
        for (int i = 0; i < bitmask.size(); i++) {
            bitmask[i].value = (custom_bitmask >> bitmask[i].bitShift) & ((1 << bitmask[i].bitSize) - 1);
        }
    }
}