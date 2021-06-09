#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.hpp"

// for convenience
using nlohmann::json;
using std::vector;

constexpr const char *kMapFile = "../data/highway_map.csv";
constexpr double max_s = 6945.554;
constexpr double kTimeDelta = 0.02;

enum class Lane: std::uint8_t {
  LEFT,
  MIDDLE,
  RIGHT,
  OFFROAD,
};

Lane DetermineLane(double d) {
    if(d > 0 && d < 4) return Lane::LEFT;
    else if(d > 4 && d < 8) return Lane::MIDDLE;
    else if(d > 8 && d < 12) return Lane::RIGHT;
    return Lane::OFFROAD;
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

  void ReadMap(const char *map_file) {
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
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
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
  }

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};



int main() {
    uWS::Hub h;

    // Waypoint map to read from
    // The max s value before wrapping around the track back to 0
    MapData map_data;
    map_data.ReadMap(kMapFile);

    Vehicle ego_vehicle;
    vector<Vehicle> other_vehicles;

    h.onMessage([&map_data](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                    size_t length, uWS::OpCode opCode) {
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

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * TODO: define a path made up of (x,y) points that the car
                     * will visit sequentially every .02 seconds
                     */

                    // Prediction
                    bool car_ahead{false};
                    bool car_left{false};
                    bool car_right{false};
                    for(size_t i = 0; i < sensor_fusion.size(); i++) {
                        // [ id, x, y, vx, vy, s, d]
                        int id = sensor_fusion[i][0];
                        int x = sensor_fusion[i][1];
                        int y = sensor_fusion[i][2];
                        int vx = sensor_fusion[i][3];
                        int vy = sensor_fusion[i][4];
                        int s = sensor_fusion[i][5];
                        int d = sensor_fusion[i][6];
                        Lane car_lane = DetermineLane(d);
                        if (car_lane == Lane::OFFROAD) continue;

                        double v = std::sqrt(vx*vx + vy*vy);
                        double car_prediction = s + v * previous_path_x.size() * kTimeDelta;
                        if(ego_vehicle_data.lane == car_lane) {
                            car_ahead |= s > ego_vehicle_data.s && s - ego_vehicle_data.s < 30;
                        } else if(ego_vehicle_data.lane > car_lane) {
                            car_left |= ego_vehicle_data.s - 30 < s && ego_vehicle_data.s + 30 > s;
                        } else if(ego_vehicle_data.lane < car_lane) {
                            car_right |= ego_vehicle_data.s - 30 < s && ego_vehicle_data.s + 30 > s;
                        }
                    }
                    // Prediction over


                    double pos_x;
                    double pos_y;
                    double angle;
                    int path_size = previous_path_x.size();

                    for (int i = 0; i < path_size; ++i) {
                      next_x_vals.push_back(previous_path_x[i]);
                      next_y_vals.push_back(previous_path_y[i]);
                    }

                    if (path_size == 0) {
                      pos_x = ego_vehicle_data.x;
                      pos_y = ego_vehicle_data.y;
                      angle = deg2rad(ego_vehicle_data.yaw);
                    } else {
                      pos_x = previous_path_x[path_size-1];
                      pos_y = previous_path_y[path_size-1];

                      double pos_x2 = previous_path_x[path_size-2];
                      double pos_y2 = previous_path_y[path_size-2];
                      angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
                    }

                    double dist_inc = 0.5;
                    for (int i = 0; i < 50-path_size; ++i) {
                      next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                      next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                      pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                      pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
                    }


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
                           char *message, size_t length) {
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