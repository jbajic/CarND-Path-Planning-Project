#include <uWS/uWS.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "traffic.hpp"
#include "map.hpp"

// for convenience
using std::vector;

constexpr const char* kMapFile = "../data/highway_map.csv";
constexpr double max_s = 6945.554;
constexpr double kTimeDelta = 0.02;
constexpr double kMaxSpeed = 50;
constexpr double kMaxAcceleration = 0.225;

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


void PredictVehicle(const nlohmann::json &sensor_fusion, traffic::Vehicle &ego_vehicle, size_t previous_path_size) {
    bool car_ahead{false};
    bool car_left{false};
    bool car_right{false};
    for (size_t car_id = 0; car_id < sensor_fusion.size();
            car_id++) {
        // [ id, x, y, vx, vy, s, d]
        int id = sensor_fusion[car_id][0];
        int x = sensor_fusion[car_id][1];
        int y = sensor_fusion[car_id][2];
        int vx = sensor_fusion[car_id][3];
        int vy = sensor_fusion[car_id][4];
        int s = sensor_fusion[car_id][5];
        int d = sensor_fusion[car_id][6];
        traffic::Lane car_lane = traffic::DetermineLane(d);
        if (car_lane == traffic::Lane::OFFROAD) continue;

        double v = std::sqrt(vx * vx + vy * vy);
        double car_prediction =
            s + v * previous_path_size * kTimeDelta;
        if (ego_vehicle.lane == car_lane) {
            car_ahead |= s > ego_vehicle.s &&
                            s - ego_vehicle.s < 30;
        } else if (ego_vehicle.lane > car_lane) {
            car_left |= ego_vehicle.s - 30 < s &&
                        ego_vehicle.s + 30 > s;
        } else if (ego_vehicle.lane < car_lane) {
            car_right |= ego_vehicle.s - 30 < s &&
                            ego_vehicle.s + 30 > s;
        }
    }
    ego_vehicle.UpdateStates(car_left, car_right);
}

int main() {
    uWS::Hub h;

    // Waypoint map to read from
    // The max s value before wrapping around the track back to 0
    MapData map_data;
    map_data.ReadMap(kMapFile);

    double ref_speed{0.0};  // mph

    h.onMessage([&map_data, &ref_speed](uWS::WebSocket<uWS::SERVER> ws,
                                        char* data, size_t length,
                                        uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message
        // event. The 4 signifies a websocket message The 2 signifies a
        // websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(data);

            if (s != "") {
                nlohmann::json j = nlohmann::json::parse(s);
                std::string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    traffic::Vehicle ego_vehicle(j[1]);

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
                    bool car_ahead{false}, car_left{false}, car_right{false};
                    for (size_t car_id = 0; car_id < sensor_fusion.size();
                         car_id++) {
                        // [ id, x, y, vx, vy, s, d]
                        int id = sensor_fusion[car_id][0];
                        int x = sensor_fusion[car_id][1];
                        int y = sensor_fusion[car_id][2];
                        int vx = sensor_fusion[car_id][3];
                        int vy = sensor_fusion[car_id][4];
                        int s = sensor_fusion[car_id][5];
                        int d = sensor_fusion[car_id][6];
                        traffic::Lane car_lane = traffic::DetermineLane(d);
                        if (car_lane == traffic::Lane::OFFROAD) continue;

                        double v = std::sqrt(vx * vx + vy * vy);
                        double car_prediction =
                            s + v * previous_path_x.size() * kTimeDelta;
                        if (ego_vehicle.lane == car_lane) {
                            car_ahead |= s > ego_vehicle.s &&
                                         s - ego_vehicle.s < 30;
                        } else if (ego_vehicle.lane > car_lane) {
                            car_left |= ego_vehicle.s - 30 < s &&
                                        ego_vehicle.s + 30 > s;
                        } else if (ego_vehicle.lane < car_lane) {
                            car_right |= ego_vehicle.s - 30 < s &&
                                         ego_vehicle.s + 30 > s;
                        }
                    }
                    // Prediction over
                    vector<double> path_x(5), path_y(5), path_s(5), path_d(5);
                    int nex_waypoints_id = NextWaypoint(ego_vehicle.x, ego_vehicle.y, ego_vehicle.yaw, map_data);
                    for(size_t i = 0; i < 5; i++) {
                        // int waypoint_id  ()
                    }
                    //Construct

                    // Behaviour planning
                    double speed_diff{0};
                    traffic::Lane lane = ego_vehicle.lane;
                    if (car_ahead) {  // Car ahead
                        if (!car_left && lane > traffic::Lane::LEFT) {
                            // if there is no car left and there is a left lane.
                            --lane;  // Change lane left.
                        } else if (!car_right && lane != traffic::Lane::RIGHT) {
                            // if there is no car right and there is a right
                            // lane.
                            ++lane;  // Change lane right.
                        } else {
                            speed_diff -= kMaxAcceleration;
                        }
                    } else {
                        if (lane != traffic::Lane::MIDDLE) {  // if we are not on the
                                                     // center lane.
                            if ((lane == traffic::Lane::LEFT && !car_right) ||
                                (lane == traffic::Lane::RIGHT && !car_left)) {
                                lane = traffic::Lane::MIDDLE;  // Back to center.
                            }
                        }
                        if (ref_speed < kMaxSpeed) {
                            speed_diff += kMaxAcceleration;
                        }
                    }

                    vector<double> ptsx;
                    vector<double> ptsy;
                    double ref_x = ego_vehicle.x;
                    double ref_y = ego_vehicle.y;
                    double ref_yaw = deg2rad(ego_vehicle.yaw);
                    if (previous_path_x.size() < 2) {
                        // There are not too many...
                        double prev_car_x =
                            ego_vehicle.x - cos(ego_vehicle.yaw);
                        double prev_car_y =
                            ego_vehicle.y - sin(ego_vehicle.yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(ego_vehicle.x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(ego_vehicle.y);
                    } else {
                        // Use the last two points.
                        ref_x = previous_path_x[previous_path_x.size() - 1];
                        ref_y = previous_path_y[previous_path_x.size() - 1];

                        double ref_x_prev =
                            previous_path_x[previous_path_x.size() - 2];
                        double ref_y_prev =
                            previous_path_y[previous_path_x.size() - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // Setting up target points in the future.
                    vector<double> next_wp0 =
                        getXY(ego_vehicle.s + 30, GetLaneDistance(lane),
                              map_data.waypoints_s, map_data.waypoints_x,
                              map_data.waypoints_y);
                    vector<double> next_wp1 =
                        getXY(ego_vehicle.s + 60, GetLaneDistance(lane),
                              map_data.waypoints_s, map_data.waypoints_x,
                              map_data.waypoints_y);
                    vector<double> next_wp2 =
                        getXY(ego_vehicle.s + 90, GetLaneDistance(lane),
                              map_data.waypoints_s, map_data.waypoints_x,
                              map_data.waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for (size_t i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = shift_x * std::cos(0 - ref_yaw) -
                                  shift_y * std::sin(0 - ref_yaw);
                        ptsy[i] = shift_x * std::sin(0 - ref_yaw) +
                                  shift_y * std::cos(0 - ref_yaw);
                    }

                    tk::spline s;
                    s.set_points(ptsx, ptsy);

                    // Output path points from previous path for continuity.
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    for (size_t i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Calculate distance y position on 30 m ahead.
                    double target_x{30.0};
                    double target_y = s(target_x);
                    double target_dist =
                        std::sqrt(target_x * target_x + target_y * target_y);

                    double x_add_on{0};
                    for (size_t i = 1; i < 50 - previous_path_x.size(); i++) {
                        ref_speed += speed_diff;
                        if (ref_speed > kMaxSpeed) {
                            ref_speed = kMaxSpeed;
                        } else if (ref_speed < kMaxAcceleration) {
                            ref_speed = kMaxAcceleration;
                        }
                        double N =
                            target_dist / (kTimeDelta * ref_speed / 2.24);
                        double x_point = x_add_on + target_x / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    // Behaviour planning over
                    nlohmann::json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

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