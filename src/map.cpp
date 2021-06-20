#include "map.hpp"
#include <sstream>
#include <fstream>

void MapData::ReadMap(const char* map_file) {
    // Load up map values for waypoint's x,y,s and d normalized normal
    // vectors
    std::ifstream in_map_(map_file, std::ifstream::in);

    std::string line;
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
