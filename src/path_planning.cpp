#include "path_planning.hpp"

#include "Eigen-3.3/Eigen/Dense"
#include "constants.hpp"
#include <vector>
#include <unordered_map>

namespace path_planning {
std::vector<std::vector<double>> GenerateTrajectory(
    const std::vector<std::vector<double>> &target_s_and_d,
    traffic::Vehicle &ego_vehicle, const double duration) {
    std::vector<double> target_s = target_s_and_d[0];
    std::vector<double> target_d = target_s_and_d[1];
    std::vector<double> current_s = {ego_vehicle.s, ego_vehicle.s_d,
                                     ego_vehicle.s_dd};
    std::vector<double> current_d = {ego_vehicle.d, ego_vehicle.d_d,
                                     ego_vehicle.d_dd};

    ego_vehicle.s_traj_coeffs = JMT(current_s, target_s, duration);
    ego_vehicle.d_traj_coeffs = JMT(current_d, target_d, duration);

    std::vector<double> s_trajectory;
    std::vector<double> d_trajectory;
    for (size_t i = 0; i < kNumOfSample; i++) {
        double t = i * duration / kNumOfSample;
        double s_val = 0, d_val = 0;
        for (size_t j = 0; j < ego_vehicle.s_traj_coeffs.size(); j++) {
            s_val += ego_vehicle.s_traj_coeffs[j] * pow(t, j);
            d_val += ego_vehicle.d_traj_coeffs[j] * pow(t, j);
        }
        s_trajectory.push_back(s_val);
        d_trajectory.push_back(d_val);
    }

    return {s_trajectory, d_trajectory};
}

std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end,
                        const double T) {
    double s_initial = start[0];
    double s_initial_der1 = start[1];
    double s_initial_der2 = start[2];
    double s_end = end[0];
    double s_end_der1 = end[1];
    double s_end_der2 = end[2];

    double a_0 = s_initial;
    double a_1 = s_initial_der1;
    double a_2 = s_initial_der2 / 2;

    Eigen::MatrixXd A(3, 3);
    Eigen::MatrixXd B(3, 1);
    A << T * T * T, T * T * T * T, T * T * T * T * T, 3 * T * T, 4 * T * T * T,
        5 * T * T * T * T, 6 * T, 12 * T * T, 20 * T * T * T;

    B << s_end -
             (s_initial + s_initial_der1 * T + 0.5 * s_initial_der2 * T * T),
        s_end_der1 - (s_initial_der1 + s_initial_der2 * T),
        s_end_der2 - s_initial_der2;
    Eigen::VectorXd sol = A.inverse() * B;

    return {a_0, a_1, a_2, sol[0], sol[1], sol[2]};
}

double CalculateCost(const std::vector<std::vector<double>> &trajectory, const std::unordered_map<int,std::vector<std::pair<double, double>>> &cars_predictions) {

}

}  // namespace path_planning