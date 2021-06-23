#pragma once
#include <vector>
#include <unordered_map>

#include "traffic.hpp"

namespace path_planning {
std::vector<std::vector<double>> GenerateTrajectory(
    const std::vector<std::vector<double>> &target_s_and_d,
    traffic::Vehicle &ego_vehicle, const double duration);

/**
 * Calculate the Jerk Minimizing Trajectory that connects the initial state
 * to the final state in time T.
 *
 * @param start - the vehicles start location given as a length three array
 *   corresponding to initial values of [s, s_dot, s_double_dot]
 * @param end - the desired end state for vehicle. Like "start" this is a
 *   length three array.
 * @param T - The duration, in seconds, over which this maneuver should occur.
 *
 * @output an array of length 6, each value corresponding to a coefficient in
 *   the polynomial:
 *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
 *
 * EXAMPLE
 *   > JMT([0, 10, 0], [10, 10, 0], 1)
 *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
 */
std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end,
                        const double T);

};  // namespace path_planning
