#pragma once

#include <map>
#include <string>
#include <vector>

#include "json.hpp"

namespace traffic {

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
    Vehicle();

    Vehicle(const nlohmann::json& json_data);

    void Init(const nlohmann::json& json_data);

    virtual ~Vehicle();

    void UpdateStates(const bool car_left, const bool car_right);

    std::vector<double> DifferentiateCoeffs(const std::vector<double> &coeffs);

    double EvaluateCoeffs(const std::vector<double> &coeffs, const double time);

    double x;
    double y;
    double s;
    double d;
    double s_d;
    double d_d;
    double s_dd;
    double d_dd;
    double yaw;
    double speed;
    Lane lane;

    std::vector<double> s_traj_coeffs, d_traj_coeffs;

   private:
    std::vector<std::string> available_states;
};

};  // namespace traffic
