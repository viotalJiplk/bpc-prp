//
// Created by root on 4/23/25.
//
#include "helper.hpp"
#include <chrono>

namespace helper {
    long getTimestamp() {
        auto timeStamp = std::chrono::steady_clock::now();
        long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
            timeStamp.time_since_epoch()
        ).count();
        return timeNow;
    }
}
