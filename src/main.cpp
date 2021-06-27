#include <uWS/uWS.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "constants.hpp"
#include "cost.hpp"
#include "helpers.hpp"
#include "json.hpp"
#include "map.hpp"
#include "path_planning.hpp"
#include "traffic.hpp"

// for convenience
using std::vector;

int GetLaneDistance(traffic::Lane lane) {
    // Return lane distance from the middle of the lane
    // e.g. every lane width is 4, therefore the middle of left lane is at 2
    // meters
    switch (lane) {
        case traffic::Lane::LEFT:
            return 2;
        case traffic::Lane::MIDDLE:
            return 6;
        case traffic::Lane::RIGHT:
            return 10;
        case traffic::Lane::OFFROAD:
            return -1;
        default:
            return -1;
    }
}

constexpr double DeriveByTime(double v1, double v2) {
    return (v1 - v2) / kTimeDelta;
}

void SetupFrenetParametersOnEgoVehicle(const vector<double>& previous_path_x,
                                       const vector<double>& previous_path_y,
                                       int previous_path_size,
                                       const Path& interpolated_points,
                                       traffic::Vehicle& ego_vehicle) {
    if (previous_path_size > 3) {
        double pos_s, s_d, s_dd, pos_d, d_d, d_dd;
        double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1, pos_x3,
            pos_y3, vel_x2, vel_y2, acc_x, acc_y;
        pos_x = previous_path_x[previous_path_size - 1];
        pos_y = previous_path_y[previous_path_size - 1];
        pos_x2 = previous_path_x[previous_path_size - 2];
        pos_y2 = previous_path_y[previous_path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);

        std::vector<double> frenet =
            helpers::GetFrenet(pos_x, pos_y, angle, interpolated_points.x,
                               interpolated_points.y, interpolated_points.s);
        pos_s = frenet[0];
        pos_d = frenet[1];

        int next_interp_waypoint_index = helpers::NextWaypoint(
            pos_x, pos_y, angle, interpolated_points.x, interpolated_points.y);

        double dx = interpolated_points.dx[next_interp_waypoint_index - 1];
        double dy = interpolated_points.dy[next_interp_waypoint_index - 1];
        double sx = -dy;
        double sy = dx;

        vel_x1 = DeriveByTime(pos_x, pos_x2);
        vel_y1 = DeriveByTime(pos_y, pos_y2);
        s_d = vel_x1 * sx + vel_y1 * sy;
        d_d = vel_x1 * dx + vel_y1 * dy;

        pos_x3 = previous_path_x[previous_path_size - 3];
        pos_y3 = previous_path_y[previous_path_size - 3];
        vel_x2 = DeriveByTime(pos_x2, pos_x3);
        vel_y2 = DeriveByTime(pos_y2, pos_y3);
        acc_x = DeriveByTime(vel_x1, vel_x2);
        acc_y = DeriveByTime(vel_y1, vel_y2);
        s_dd = acc_x * sx + acc_y * sy;
        d_dd = acc_x * dx + acc_y * dy;
        ego_vehicle.s = pos_s;
        ego_vehicle.s_d = s_d;
        ego_vehicle.s_dd = s_dd;
        ego_vehicle.d = pos_d;
        ego_vehicle.d_d = d_d;
        ego_vehicle.d_dd = d_dd;
    }

    std::cout << "Ego vehicle\n";
    std::cout << "(x,y,s,d)=" << "( " << ego_vehicle.x <<", " << ego_vehicle.y << ", " << ego_vehicle.s << ", " << ego_vehicle.d << ")\n";
    std::cout << "(s_d,s_dd,d_d,d_dd)=" << "( " << ego_vehicle.s_d <<", " << ego_vehicle.s_dd << ", " << ego_vehicle.d_d << ", " << ego_vehicle.d_dd << ")\n";
}

int main() {
    uWS::Hub h;

    // Waypoint map to read from
    // The max s value before wrapping around the track back to 0
    MapData map_data;
    map_data.ReadMap(kMapFile);
    traffic::Vehicle ego_vehicle;
    static int iteration = 0;
    //std::freopen( "output.txt", "w", stdout );
    //std::freopen( "error.txt", "w", stderr );

    h.onMessage([&map_data, &ego_vehicle](uWS::WebSocket<uWS::SERVER> ws,
                                          char* data, size_t length,
                                          uWS::OpCode opCode) {
        std::cout << "This is " << iteration << std::endl;
        iteration++;
        // "42" at the start of the message means there's a websocket message
        // event. The 4 signifies a websocket message The 2 signifies a
        // websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = helpers::hasData(data);
            if (s != "") {
                nlohmann::json j = nlohmann::json::parse(s);
                std::string event = j[0].get<std::string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    // Main car's localization Data
                    ego_vehicle.Init(j[1]);

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same
                    // side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    /**
                     * TODO: define a path made up of (x,y) points that the car
                     * will visit sequentially every .02 seconds
                     */

                    // Prediction
                    const int previous_path_size =
                        std::min(25, static_cast<int>(previous_path_x.size()));
                    const double trajectory_start_time =
                        previous_path_size * kTimeDelta;
                    const double duration =
                        kNumOfSample * dt - previous_path_size * kTimeDelta;

                    // Define next waypoints
                    Path path;
                    int next_waypoints_id =
                        helpers::NextWaypoint(ego_vehicle.x, ego_vehicle.y,
                                              ego_vehicle.yaw, map_data);
                    const double next_waypoint_s =
                        map_data.waypoints_s[next_waypoints_id];
                    int num_waypoints = map_data.NumberOfPoints();
                    for (int i = -5; i < 5; i++) {
                        int waypoint_id =
                            (next_waypoints_id + i) % num_waypoints;
                        if (waypoint_id < 0) {
                            waypoint_id += num_waypoints;
                        }

                        // make points continuos for spline functionality
                        double current_s = map_data.waypoints_s[waypoint_id];
                        if (i < 0 && current_s > next_waypoint_s) {
                            current_s -= kTrackLength;
                        }
                        if (i > 0 && current_s < next_waypoint_s) {
                            current_s += kTrackLength;
                        }

                        path.x.push_back(map_data.waypoints_x[waypoint_id]);
                        path.y.push_back(map_data.waypoints_y[waypoint_id]);
                        path.s.push_back(current_s);
                        path.dx.push_back(map_data.waypoints_dx[waypoint_id]);
                        path.dy.push_back(map_data.waypoints_dy[waypoint_id]);
                    }

                    std::cout << "END Define next waypoints\n";
                    // Finish defining next waypoints
                    // Interpolate points
                    const double interpolation_coeff{0.5};
                    const int num_interpolation_points =
                        (path.s[path.s.size() - 1] - path.s[0]) /
                        interpolation_coeff;
                    Path interpolated_points;

                    for (int i = 0; i < num_interpolation_points; i++) {
                        interpolated_points.s.push_back(
                            path.s[0] + i * interpolation_coeff);
                    }
                    interpolated_points.x = helpers::Interpolate(
                        path.s, path.x, interpolation_coeff,
                        num_interpolation_points);
                    interpolated_points.y = helpers::Interpolate(
                        path.s, path.y, interpolation_coeff,
                        num_interpolation_points);
                    interpolated_points.dx = helpers::Interpolate(
                        path.s, path.dx, interpolation_coeff,
                        num_interpolation_points);
                    interpolated_points.dy = helpers::Interpolate(
                        path.s, path.dy, interpolation_coeff,
                        num_interpolation_points);
                    // Finish interpolation

                    // Determine ego vehicle
                    SetupFrenetParametersOnEgoVehicle(
                        previous_path_x, previous_path_y, previous_path_size,
                        interpolated_points, ego_vehicle);

                    vector<traffic::OtherVehicle> vehicles;
                    std::unordered_map<int,
                                       std::vector<std::pair<double, double>>>
                        cars_predictions;
                    for (const auto& car_data : sensor_fusion) {
                        // This is what car_data contains [ id, x, y, vx, vy, s,
                        // d]
                        traffic::OtherVehicle other_car(car_data);
                        vehicles.push_back(other_car);
                        cars_predictions[other_car.id] =
                            other_car.GeneratePrediction(trajectory_start_time,
                                                         duration);
                    }

                    bool car_ahead{false}, car_left{false}, car_right{false};
                    for (const auto& vehicle : vehicles) {
                        double distance_from_ego_s{
                            std::abs(vehicle.s - ego_vehicle.s)};
                        if (distance_from_ego_s < kTrackingDistance) {
                            std::cout << "Vehicle " << vehicle.id << " is "
                                      << distance_from_ego_s << " m "
                                      << " from ego vehicle\n";
                            double distance_from_ego_d =
                                vehicle.d - ego_vehicle.d;

                            if (distance_from_ego_d > 2 &&
                                distance_from_ego_d < 6) {
                                car_right = true;
                            } else if (distance_from_ego_d < -2 &&
                                       distance_from_ego_d > -6) {
                                car_left = true;
                            } else if (distance_from_ego_d > -2 &&
                                       distance_from_ego_d < 2) {
                                car_ahead = true;
                            }
                        }
                        if (car_left && car_ahead && car_right) break;
                    }
                    ego_vehicle.UpdateStates(car_left, car_right);
                    std::cout << "END prediction\n";
                    // Prediction over

                    // Behaviour planning START
                    vector<vector<double>> best_frenet_trajectory, best_target;
                    double best_cost = std::numeric_limits<double>::max();
                    std::string best_state = "";
                    static std::vector<std::pair<
                        double, path_planning::cost::cost_function_base>>
                        cost_functions = {
                            {999999.0, path_planning::cost::CollisionCost},
                            {10, path_planning::cost::BufferCost},
                            {1000, path_planning::cost::InLaneBufferCost},
                            {10000, path_planning::cost::EfficiencyCost},
                            {100, path_planning::cost::NotMiddleLaneCost},
                        };
                    for (auto& state : ego_vehicle.GetStates()) {
                        vector<vector<double>> target_s_and_d =
                            GetTargetForState(state, cars_predictions,
                                              ego_vehicle, duration, car_ahead);

                        vector<vector<double>> trajectory =
                            path_planning::GenerateTrajectory(
                                target_s_and_d, ego_vehicle, duration);
                        double cost = path_planning::cost::CalculateCost(
                            trajectory, cars_predictions, cost_functions);
                        if (cost < best_cost) {
                            best_cost = cost;
                            best_frenet_trajectory = trajectory;
                            best_target = target_s_and_d;
                            best_state = state;
                        }
                    }

                    double prev_s =
                        ego_vehicle.s - ego_vehicle.s_d * kTimeDelta;
                    std::vector<double> coarse_s_trajectory,
                        coarse_x_trajectory, coarse_y_traj,
                        interpolated_s_trajectory, interpolated_x_trajectory,
                        interpolated_y_trajectory;
                    if (previous_path_size >= 2) {
                        coarse_s_trajectory.push_back(prev_s);
                        coarse_x_trajectory.push_back(
                            previous_path_x[previous_path_size - 2]);
                        coarse_y_traj.push_back(
                            previous_path_y[previous_path_size - 2]);
                        coarse_s_trajectory.push_back(ego_vehicle.s);
                        coarse_x_trajectory.push_back(
                            previous_path_x[previous_path_size - 1]);
                        coarse_y_traj.push_back(
                            previous_path_y[previous_path_size - 1]);
                    } else {
                        double prev_s = ego_vehicle.s - 1;
                        double prev_x = ego_vehicle.x - cos(ego_vehicle.angle);
                        double prev_y = ego_vehicle.y - sin(ego_vehicle.angle);
                        coarse_s_trajectory.push_back(prev_s);
                        coarse_x_trajectory.push_back(prev_x);
                        coarse_y_traj.push_back(prev_y);
                        coarse_s_trajectory.push_back(ego_vehicle.s);
                        coarse_x_trajectory.push_back(ego_vehicle.x);
                        coarse_y_traj.push_back(ego_vehicle.y);
                    }

                    double target_s1 = ego_vehicle.s + 30;
                    double target_d1 = best_target[1][0];
                    std::vector<double> target_xy1 = helpers::getXY(
                        target_s1, target_d1, interpolated_points.s,
                        interpolated_points.x, interpolated_points.y);
                    double target_x1 = target_xy1[0];
                    double target_y1 = target_xy1[1];

                    coarse_s_trajectory.push_back(target_s1);
                    coarse_x_trajectory.push_back(target_x1);
                    coarse_y_traj.push_back(target_y1);

                    double target_s2 = target_s1 + 30;
                    double target_d2 = target_d1;
                    std::vector<double> target_xy2 = helpers::getXY(
                        target_s2, target_d2, interpolated_points.s,
                        interpolated_points.x, interpolated_points.y);
                    double target_x2 = target_xy2[0];
                    double target_y2 = target_xy2[1];
                    coarse_s_trajectory.push_back(target_s2);
                    coarse_x_trajectory.push_back(target_x2);
                    coarse_y_traj.push_back(target_y2);

                    double target_s_d = best_target[0][1];
                    double current_s = ego_vehicle.s;
                    double current_v = ego_vehicle.s_d;
                    double current_a = ego_vehicle.s_dd;
                    for (int i = 0; i < (kNumOfPoints - previous_path_size);
                         i++) {
                        double v_incr, a_incr;
                        if (std::abs(target_s_d - current_v) < 2 * 0.125) {
                            v_incr = 0;
                        } else {
                            v_incr = (target_s_d - current_v) /
                                     (fabs(target_s_d - current_v)) * 0.125;
                        }
                        current_v += v_incr;
                        current_v = std::min(current_v, kTrackLengthpeedMS);
                        current_s += current_v * kTimeDelta;
                        interpolated_s_trajectory.push_back(current_s);
                    }
                    // Behaviour planning END
                    // // Behaviour planning over
                    std::cout << "START Generate final path\n";
                    std::cout << "Interpolate x trajectory s: "
                              << coarse_s_trajectory.size()
                              << " x: " << coarse_x_trajectory.size()
                              << " y: " << coarse_y_traj.size() << "\n";
                    interpolated_x_trajectory = helpers::Interpolate(
                        coarse_s_trajectory, coarse_x_trajectory,
                        interpolated_s_trajectory);
                    std::cout << "Interpolate y trajectory\n";
                    interpolated_y_trajectory =
                        helpers::Interpolate(coarse_s_trajectory, coarse_y_traj,
                                             interpolated_s_trajectory);

                    std::vector<double> next_x_vals, next_y_vals;
                    for (size_t i = 0; i < previous_path_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    for (size_t i = 0; i < interpolated_x_trajectory.size();
                         i++) {
                        next_x_vals.push_back(interpolated_x_trajectory[i]);
                        next_y_vals.push_back(interpolated_y_trajectory[i]);
                    }
                    std::cout << "Next values: \n";
                    for (int i = 0; i < next_x_vals.size(); i++) {
                        std::cout << next_x_vals[i] << ", " << next_y_vals[i]
                                  << "\n";
                    }
                    nlohmann::json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    std::cout << "END Generate final path\n";

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    });    // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}