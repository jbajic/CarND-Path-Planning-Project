#include "helpers.hpp"

#include <math.h>

#include <string>
#include <vector>

#include "map.hpp"
#include "spline.h"

namespace helpers {

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &points_x,
                    const vector<double> &points_y) {
    double closestLen = 100000;  // large number
    int closestWaypoint = 0;

    for (int i = 0; i < points_x.size(); ++i) {
        double map_x = points_x[i];
        double map_y = points_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int ClosestWaypoint(double x, double y, const MapData &map_data) {
    return ClosestWaypoint(x, y, map_data.waypoints_x, map_data.waypoints_y);
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta,
                 const vector<double> &points_x,
                 const vector<double> &points_y) {
    int closestWaypoint = ClosestWaypoint(x, y, points_x, points_y);

    double map_x = points_x[closestWaypoint];
    double map_y = points_y[closestWaypoint];
    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 2) {
        ++closestWaypoint;
        if (closestWaypoint == points_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const MapData &map_data) {
    return NextWaypoint(x, y, theta, map_data.waypoints_x,
                        map_data.waypoints_y);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> GetFrenet(double x, double y, double theta,
                         const vector<double> &points_x,
                         const vector<double> &points_y,
                         const vector<double> &points_s) {
    int next_wp = NextWaypoint(x, y, theta, points_x, points_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = points_x.size() - 1;
    }

    double n_x = points_x[next_wp] - points_x[prev_wp];
    double n_y = points_y[next_wp] - points_y[prev_wp];
    double x_x = x - points_x[prev_wp];
    double x_y = y - points_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - points_x[prev_wp];
    double center_y = 2000 - points_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = points_s[0];
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += distance(points_x[i], points_y[i], points_x[i + 1],
                             points_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

vector<double> GetFrenet(double x, double y, double theta,
                         const MapData &map_data) {
    return GetFrenet(x, y, theta, map_data.waypoints_x, map_data.waypoints_y,
                     map_data.waypoints_s);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading =
        atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

vector<double> Interpolate(const vector<double> &points_x,
                           const vector<double> &points_y, const double step,
                           const size_t output_size) {
    assert(points_x.size() == points_y.size());

    tk::spline s;
    s.set_points(points_x, points_y);
    std::vector<double> output;
    for (size_t i = 0; i < output_size; i++) {
        output.push_back(s(points_x[0] + i * step));
    }
    return output;
}

vector<double> Interpolate(const vector<double> &points_x,
                           const vector<double> &points_y,
                           const vector<double> &coefficients) {
    assert(points_x.size() == points_y.size());

    tk::spline s;
    s.set_points(points_x, points_y);
    std::vector<double> output;
    for (auto c : coefficients) {
        output.push_back(s(c));
    }
    return output;
}
}  // namespace helpers