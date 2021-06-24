#include "traffic.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "constants.hpp"
#include "helpers.hpp"

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

Vehicle::Vehicle(const nlohmann::json& json_data) { Init(json_data); }

void Vehicle::Init(const nlohmann::json& json_data) {
    x = json_data["x"];
    y = json_data["y"];
    s = json_data["s"];
    d = json_data["d"];
    yaw = json_data["yaw"];
    angle = helpers::deg2rad(yaw);
    speed = json_data["speed"];
    speed *= 0.44704;
    lane = DetermineLane(d);
    available_states.clear();
}

std::vector<double> Vehicle::DifferentiateCoeffs(
    const std::vector<double>& coefficients) {
    std::vector<double> diff_coefficients;
    for (int i = 1; i < coefficients.size(); i++) {
        diff_coefficients.push_back(i * coefficients[i]);
    }
    return diff_coefficients;
}

double Vehicle::EvaluateCoeffs(const std::vector<double>& coefficients,
                               const double time) {
    double evaluation{0};
    for (int i = 0; i < coefficients.size(); i++) {
        evaluation += coefficients[i] * std::pow(time, i);
    }
    return evaluation;
}

std::vector<std::string> Vehicle::GetStates() const {
    return available_states;
};

void Vehicle::UpdateStates(const bool car_left, const bool car_right) {
    available_states.push_back("KL");

    if (!car_left && lane > Lane::LEFT) {
        available_states.push_back("LCL");
    }
    if (!car_right && lane > Lane::RIGHT) {
        available_states.push_back("LCR");
    }
}

OtherVehicle::OtherVehicle(const nlohmann::json& json_data)
    : id{json_data[0]},
      x{json_data[1]},
      y{json_data[2]},
      vx{json_data[3]},
      vy{json_data[4]},
      s{json_data[5]},
      d{json_data[6]} {
    speed = std::sqrt(vx * vx + vy * vy);
}

std::vector<std::pair<double, double>> OtherVehicle::GeneratePrediction(
    const double traj_start_time, const double duration) {
    std::vector<std::pair<double, double>> prediction;
    for (size_t i = 0; i < kNumOfSample; i++) {
        double t = traj_start_time + (i * duration / kNumOfSample);
        double new_s = s + speed * t;
        prediction.emplace_back(new_s, d);
    }
    return prediction;
}

std::vector<std::vector<double>> GetTargetForState(
    const std::string& state,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>&
        cars_predictions,
    const Vehicle& ego_vehicle, const double duration,
    const bool car_just_ahead) {
    int current_lane = ego_vehicle.d / 4;
    int target_lane;
    double target_d{0}, target_d_d{0}, target_d_dd{0};
    double target_s{0}, target_s_d{0}, target_s_dd{0};

    target_s_d = kMaxSpeedMS;
    target_s = ego_vehicle.s + (ego_vehicle.s_d + target_s_d) / 2 * duration;

    if (state.compare("KL") == 0) {
        target_d = static_cast<double>(current_lane) * 4 + 2;
        target_lane = target_d / 4;
    } else if (state.compare("LCL") == 0) {
        target_d = (static_cast<double>(current_lane) - 1) * 4 + 2;
        target_lane = target_d / 4;
    } else if (state.compare("LCR") == 0) {
        target_d = (static_cast<double>(current_lane) + 1) * 4 + 2;
        target_lane = target_d / 4;
    }
    std::cout << "Target lane " << target_lane << "\n";
    // target_lane = target_d / 4;

    std::vector<double> leading_vehicle_s_and_s_d =
        GetLeadingVehicleDataForLane(target_lane, cars_predictions, ego_vehicle,
                                     duration);
    double leading_vehicle_s = leading_vehicle_s_and_s_d[0];
    if (leading_vehicle_s - target_s < kTrackingDistance &&
        leading_vehicle_s > ego_vehicle.s) {
        target_s_d = leading_vehicle_s_and_s_d[1];

        if (std::abs(leading_vehicle_s - target_s) < 0.5 * kTrackingDistance) {
            target_s_d -= 1;
        }
    }
    target_s = leading_vehicle_s - kTrackingDistance;
    if (car_just_ahead) {
        target_s_d = 0.0;
    }

    return {{target_s, target_s_d, target_s_dd},
            {target_d, target_d_d, target_d_dd}};
}

std::vector<double> GetLeadingVehicleDataForLane(
    const int target_lane,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>&
        cars_predictions,
    const Vehicle& ego_vehicle, const double duration) {
    double nearest_leading_vehicle_speed{0},
        nearest_leading_vehicle_distance = std::numeric_limits<double>::max();
    for (const auto& [id, prediction] : cars_predictions) {
        // vector<vector<double>> pred_traj = prediction.second;
        // int pred_lane = pred_traj[0][1] / 4;
        int pred_lane = prediction[0].first / 4;
        if (pred_lane == target_lane) {
            double start_s = prediction[0].first;
            double predicted_end_s = prediction[prediction.size() - 1].first;
            double next_to_last_s = prediction[prediction.size() - 2].first;
            double dt = duration / kNumOfSample;
            double predicted_s_d = (predicted_end_s - next_to_last_s) / dt;
            if (predicted_end_s < nearest_leading_vehicle_distance &&
                start_s > ego_vehicle.s) {
                nearest_leading_vehicle_distance = predicted_end_s;
                nearest_leading_vehicle_speed = predicted_s_d;
            }
        }
    }
    return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
}

};  // namespace traffic