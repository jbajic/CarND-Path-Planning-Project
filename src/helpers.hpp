#pragma once
#include <math.h>

#include <string>
#include <vector>

#include "map.hpp"
#include "spline.h"

namespace helpers {

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
constexpr double deg2rad(double x) { return x * pi() / 180; }
constexpr double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
constexpr double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &points_x,
                    const vector<double> &points_y);

int ClosestWaypoint(double x, double y, const MapData &map_data);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta,
                 const vector<double> &points_x,
                 const vector<double> &points_y);

int NextWaypoint(double x, double y, double theta, const MapData &map_data);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> GetFrenet(double x, double y, double theta,
                         const vector<double> &points_x,
                         const vector<double> &points_y,
                         const vector<double> &points_s);

vector<double> GetFrenet(double x, double y, double theta,
                         const MapData &map_data);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);

vector<double> Interpolate(const vector<double> &points_x,
                           const vector<double> &points_y, const double step,
                           const size_t output_size);

vector<double> Interpolate(const vector<double> &points_x,
                           const vector<double> &points_y,
                           const vector<double> &coefficients);
}  // namespace helpers