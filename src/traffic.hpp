#pragma once

#include <map>
#include <string>
#include <unordered_map>
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

struct OtherVehicle {
    OtherVehicle(const nlohmann::json& json_data);

    std::vector<std::pair<double, double>> GeneratePrediction(
        const double traj_start_time, const double duration);

    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;
};

class Vehicle {
   public:
    Vehicle();

    Vehicle(const nlohmann::json& json_data);

    void Init(const nlohmann::json& json_data);

    virtual ~Vehicle();

    std::vector<std::string> GetStates() const;

    void UpdateStates(const bool car_left, const bool car_right);

    std::vector<double> DifferentiateCoeffs(const std::vector<double>& coeffs);

    double EvaluateCoeffs(const std::vector<double>& coeffs, const double time);

    double x;
    double y;
    double s;
    double d;
    double s_d;
    double d_d;
    double s_dd;
    double d_dd;
    double yaw;
    double angle;
    double speed;
    Lane lane;

    std::vector<double> s_traj_coeffs, d_traj_coeffs;

   private:
    std::vector<std::string> available_states;
};

std::vector<std::vector<double>> GetTargetForState(
    const std::string& state,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>&
        cars_predictions,
    const Vehicle& ego_vehicle, const double duration,
    const bool car_just_ahead);

std::vector<double> GetLeadingVehicleDataForLane(
    const int target_lane,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>&
        cars_predictions,
    const Vehicle& ego_vehicle, const double duration);

};  // namespace traffic
