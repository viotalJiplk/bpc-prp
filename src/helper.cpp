// helper.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for timestamps operations.


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
