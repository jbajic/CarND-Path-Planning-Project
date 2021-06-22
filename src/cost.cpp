#include "cost.hpp"

#include <cmath>
#include <limits>

#include "constants.hpp"

namespace path_planning {
namespace cost {

/*
 * A function that returns a value between 0 and 1 for x in the
 * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
 * Useful for cost functions.
 * Taken from Trajectory exercise...
 */
constexpr double logistic(double value) {
    return 2.0 / (1 + std::exp(-value)) - 1.0;
}

constexpr double dt = 0.2;
constexpr double kMaxObservableDistance = 120;

double DistanceOfCarInTrajectory(
    const std::vector<std::vector<double>> &trajectory,
    const std::vector<std::pair<double, double>> &cars_predictions) {
    double closest_car_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < kNumOfSample; i++) {
        double current_distance =
            sqrt(pow(trajectory[0][i] - cars_predictions[i].first, 2.0) +
                 pow(trajectory[1][i] - cars_predictions[i].second, 2));
        if (current_distance < closest_car_distance) {
            closest_car_distance = current_distance;
        }
    }
    return closest_car_distance;
}

double ClosesDistanceOfAnyCarInTrajectory(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    double closest_car_distance = std::numeric_limits<double>::max();
    for (const auto &[id, car_prediction] : cars_predictions) {
        double current_distance =
            DistanceOfCarInTrajectory(trajectory, car_prediction);
        if (current_distance < closest_car_distance) {
            closest_car_distance = current_distance;
        }
    }
    return closest_car_distance;
}

double ClosesDistanceOfAnyCarInLaneTrajectory(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    double closest_car_distance = std::numeric_limits<double>::max();
    for (const auto &[id, car_prediction] : cars_predictions) {
        double final_d = trajectory[1][trajectory[1].size() - 1];
        int lane = final_d / 4;
        double pred_final_d = car_prediction[car_prediction.size() - 1].second;
        int pred_lane = pred_final_d / 4;
        if (lane == pred_lane) {
            double current_distance =
                DistanceOfCarInTrajectory(trajectory, car_prediction);
            if (current_distance < closest_car_distance &&
                current_distance < kMaxObservableDistance) {
                closest_car_distance = current_distance;
            }
        }
    }
    return closest_car_distance;
}

std::vector<double> VelocitiesForTrajectory(
    const std::vector<std::vector<double>> &trajectory) {
    std::vector<double> velocities(trajectory.size());
    for (size_t i = 1; i < trajectory.size(); i++) {
        velocities.emplace_back((trajectory[0][i] - trajectory[0][i - 1]) / dt);
    }
    return velocities;
}

double CollisionCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    assert(trajectory.size() == 2);
    double nearest =
        ClosesDistanceOfAnyCarInTrajectory(trajectory, cars_predictions);
    if (nearest < 2 * kVehicleRadius) {
        return 1;
    }
    return 0;
}

double BufferCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    assert(trajectory.size() == 2);
    double nearest =
        ClosesDistanceOfAnyCarInTrajectory(trajectory, cars_predictions);
    return logistic(2 * kVehicleRadius / nearest);
}

double InLaneBufferCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    assert(trajectory.size() == 2);
    double nearest =
        ClosesDistanceOfAnyCarInLaneTrajectory(trajectory, cars_predictions);
    return logistic(2 * kVehicleRadius / nearest);
}

double EfficiencyCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    assert(trajectory.size() == 2);
    std::vector<double> s_dot_traj = VelocitiesForTrajectory(trajectory);
    double final_s_dot, total = 0;
    final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
    return logistic((kMaxSpeed - final_s_dot) / kMaxSpeed);
}

double NotMiddleLaneCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions) {
    assert(trajectory.size() == 2);
    double end_d = trajectory[1][trajectory[1].size() - 1];
    return logistic(pow(end_d - 6, 2));
}
}  // namespace cost
}  // namespace path_planning
