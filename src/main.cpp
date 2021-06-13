#include <uWS/uWS.h>
#include "spline.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.hpp"

// for convenience
using nlohmann::json;
using std::vector;

constexpr const char* kMapFile = "../data/highway_map.csv";
constexpr double max_s = 6945.554;
constexpr double kTimeDelta = 0.02;
constexpr double kMaxSpeed = 50;
constexpr double kMaxAcceleration = 0.225;

enum class Lane : std::uint8_t {
    LEFT,
    MIDDLE,
    RIGHT,
    OFFROAD,
};

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

int GetLaneDistance(Lane lane) {
    // Return lane distance from the middle of the lane
    // e.g. every lane width is 4, therefore the middle of left lane is at 2 meters
    switch (lane) {
        case Lane::LEFT:
            return 2;
        case Lane::MIDDLE:
            return 6;
        case Lane::RIGHT:
            return 10;
        case Lane::OFFROAD:
            return -1;
        default:
            return -1;
    }
}

struct VehicleData {
    VehicleData(nlohmann::json json_data) {
        x = json_data["x"];
        y = json_data["y"];
        s = json_data["s"];
        d = json_data["d"];
        yaw = json_data["yaw"];
        speed = json_data["speed"];
        lane = DetermineLane(d);
    }

    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    Lane lane;
};

struct MapData {
    MapData() = default;

    void ReadMap(const char* map_file) {
        // Load up map values for waypoint's x,y,s and d normalized normal
        // vectors
        std::ifstream in_map_(map_file, std::ifstream::in);

        string line;
        while (getline(in_map_, line)) {
            std::istringstream iss(line);
            double x;
            double y;
            float s;
            float d_x;
            float d_y;
            iss >> x;
            iss >> y;
            iss >> s;
            iss >> d_x;
            iss >> d_y;
            waypoints_x.push_back(x);
            waypoints_y.push_back(y);
            waypoints_s.push_back(s);
            waypoints_dx.push_back(d_x);
            waypoints_dy.push_back(d_y);
        }
    }

    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;
};

int main() {
    uWS::Hub h;

    // Waypoint map to read from
    // The max s value before wrapping around the track back to 0
    MapData map_data;
    map_data.ReadMap(kMapFile);

    vector<Vehicle> other_vehicles;
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
                nlohmann::json j = json::parse(s);

                std::string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    VehicleData ego_vehicle_data(j[1]);

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
                        Lane car_lane = DetermineLane(d);
                        if (car_lane == Lane::OFFROAD) continue;

                        double v = std::sqrt(vx * vx + vy * vy);
                        double car_prediction =
                            s + v * previous_path_x.size() * kTimeDelta;
                        if (ego_vehicle_data.lane == car_lane) {
                            car_ahead |= s > ego_vehicle_data.s &&
                                         s - ego_vehicle_data.s < 30;
                        } else if (ego_vehicle_data.lane > car_lane) {
                            car_left |= ego_vehicle_data.s - 30 < s &&
                                        ego_vehicle_data.s + 30 > s;
                        } else if (ego_vehicle_data.lane < car_lane) {
                            car_right |= ego_vehicle_data.s - 30 < s &&
                                         ego_vehicle_data.s + 30 > s;
                        }
                    }
                    // Prediction over

                    // Behaviour planning
                    double speed_diff{0};
                    Lane lane = ego_vehicle_data.lane;
                    if (car_ahead) {  // Car ahead
                        if (!car_left && lane > Lane::LEFT) {
                            // if there is no car left and there is a left lane.
                            --lane;  // Change lane left.
                        } else if (!car_right && lane != Lane::RIGHT) {
                            // if there is no car right and there is a right
                            // lane.
                            ++lane;  // Change lane right.
                        } else {
                            speed_diff -= kMaxAcceleration;
                        }
                    } else {
                        if (lane != Lane::MIDDLE) {  // if we are not on the
                                                     // center lane.
                            if ((lane == Lane::LEFT && !car_right) ||
                                (lane == Lane::RIGHT && !car_left)) {
                                lane = Lane::MIDDLE;  // Back to center.
                            }
                        }
                        if (ref_speed < kMaxSpeed) {
                            speed_diff += kMaxAcceleration;
                        }
                    }

                    vector<double> ptsx;
                    vector<double> ptsy;
                    double ref_x = ego_vehicle_data.x;
                    double ref_y = ego_vehicle_data.y;
                    double ref_yaw = deg2rad(ego_vehicle_data.yaw);
                    if (previous_path_x.size() < 2) {
                        // There are not too many...
                        double prev_car_x = ego_vehicle_data.x - cos(ego_vehicle_data.yaw);
                        double prev_car_y = ego_vehicle_data.y - sin(ego_vehicle_data.yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(ego_vehicle_data.x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(ego_vehicle_data.y);
                   } else {
                        // Use the last two points.
                        ref_x = previous_path_x[previous_path_x.size() - 1];
                        ref_y = previous_path_y[previous_path_x.size() - 1];

                        double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
                        double ref_y_prev = previous_path_y[previous_path_x.size() - 2];
                        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // Setting up target points in the future.
                    vector<double> next_wp0 = getXY(ego_vehicle_data.s + 30, GetLaneDistance(lane), map_data.waypoints_s, map_data.waypoints_x, map_data.waypoints_y);
                    vector<double> next_wp1 = getXY(ego_vehicle_data.s + 60, GetLaneDistance(lane), map_data.waypoints_s, map_data.waypoints_x, map_data.waypoints_y);
                    vector<double> next_wp2 = getXY(ego_vehicle_data.s + 90, GetLaneDistance(lane), map_data.waypoints_s, map_data.waypoints_x, map_data.waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for (size_t i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = shift_x * std::cos(0 - ref_yaw) - shift_y * std::sin(0 - ref_yaw);
                        ptsy[i] = shift_x * std::sin(0 - ref_yaw) + shift_y * std::cos(0 - ref_yaw);
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
                    double target_dist = std::sqrt(target_x*target_x + target_y*target_y);

                    double x_add_on{0};
                     for(size_t i = 1; i < 50 - previous_path_x.size(); i++ ) {
                        ref_speed += speed_diff;
                        if ( ref_speed > kMaxSpeed ) {
                            ref_speed = kMaxSpeed;
                        } else if ( ref_speed < kMaxAcceleration ) {
                            ref_speed = kMaxAcceleration;
                        }
                        double N = target_dist/(kTimeDelta*ref_speed/2.24);
                        double x_point = x_add_on + target_x/N;
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
                    json msgJson;
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