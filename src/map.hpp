#pragma once

#include <vector>
#include <string>

struct MapData {
    MapData() = default;

    void ReadMap(const char* map_file);

    size_t Size() const {
        return waypoints_x.size();
    }

    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;
};
