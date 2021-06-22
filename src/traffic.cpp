#include "traffic.hpp"
namespace traffic {
Lane& operator++(Lane& lane) {
    switch (lane) {
        case Lane::LEFT:
            return lane = Lane::MIDDLE;
        case Lane::MIDDLE:
            return lane = Lane::RIGHT;
        case Lane::RIGHT:
            return lane = Lane::OFFROAD;
        case Lane::OFFROAD:
            return lane = Lane::OFFROAD;
        default:
            return lane = Lane::OFFROAD;
    }
}

Lane& operator--(Lane& lane) {
    switch (lane) {
        case Lane::LEFT:
            return lane = Lane::OFFROAD;
        case Lane::MIDDLE:
            return lane = Lane::LEFT;
        case Lane::RIGHT:
            return lane = Lane::MIDDLE;
        case Lane::OFFROAD:
            return lane = Lane::OFFROAD;
        default:
            return lane = Lane::OFFROAD;
    }
}

Lane DetermineLane(double d) {
    if (d > 0 && d < 4)
        return Lane::LEFT;
    else if (d > 4 && d < 8)
        return Lane::MIDDLE;
    else if (d > 8 && d < 12)
        return Lane::RIGHT;
    return Lane::OFFROAD;
}

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

Vehicle::Vehicle(const nlohmann::json& json_data) {
   Init(json_data);
}

void Vehicle::Init(const nlohmann::json& json_data) {
    x = json_data["x"];
    y = json_data["y"];
    s = json_data["s"];
    d = json_data["d"];
    yaw = json_data["yaw"];
    speed = json_data["speed"];
    lane = DetermineLane(d);
}

std::vector<double> Vehicle::DifferentiateCoeffs(const std::vector<double> &coeffs) {
  std::vector<double> diff_coeffs;
  for (size_t i = 1; i < coeffs.size(); i++) {
    diff_coeffs.push_back(i * coeffs[i]);
  }
  return diff_coeffs;
}

double Vehicle::EvaluateCoeffs(const std::vector<double> &coeffs, const double time) {
  double eval{0};
  for (int i = 0; i < coeffs.size(); i++) {
    eval += coeffs[i] * pow(time, i);
  }
  return eval;
}

void Vehicle::UpdateStates(const bool car_left, const bool car_right) {
    available_states.push_back("KP");

    if (!car_left && lane > Lane::LEFT) {
        available_states.push_back("PLCL");
        available_states.push_back("LCL");
    }
    if (!car_right && lane > Lane::RIGHT) {
        available_states.push_back("PLCR");
        available_states.push_back("LCR");
    }
}

};  // namespace traffic