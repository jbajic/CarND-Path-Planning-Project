#pragma once

#include <map>
#include <string>
#include <vector>

#include "json.hpp"

namespace traffic {

using std::map;
using std::string;
using std::vector;

enum class Lane : std::uint8_t {
    LEFT,
    MIDDLE,
    RIGHT,
    OFFROAD,
};

// TODO implement via templates
// https://stackoverflow.com/questions/15451382/implementation-of-operators-for-enum-class
Lane& operator++(Lane& lane);

Lane& operator--(Lane& lane);

Lane DetermineLane(double d);

class Vehicle {
   public:
    Vehicle(const nlohmann::json& json_data);

    void UpdateStates(const bool car_left, const bool car_right);

    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    Lane lane;

   private:
    vector<std::string> available_states;
};

};  // namespace traffic
