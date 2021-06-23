#pragma once

#include <unordered_map>
#include <vector>

#include "traffic.hpp"

namespace path_planning {

namespace cost {

constexpr double kVehicleRadius = 1.25;

using cost_function_base = std::function<double(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions)>;

double CollisionCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &predictions);

double BufferCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &predictions);

double InLaneBufferCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &predictions);

double EfficiencyCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &predictions);

double NotMiddleLaneCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &predictions);

double CalculateCost(
    const std::vector<std::vector<double>> &trajectory,
    const std::unordered_map<int, std::vector<std::pair<double, double>>>
        &cars_predictions,
    std::vector<std::pair<double, cost_function_base>> &cost_functions);

}  // namespace cost

}  // namespace path_planning
