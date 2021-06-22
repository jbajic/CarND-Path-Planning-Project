#pragma once

#include <unordered_map>
#include <vector>

#include "traffic.hpp"

namespace path_planning {

namespace cost {

constexpr double kVehicleRadius = 1.25;

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

}  // namespace cost

}  // namespace path_planning
