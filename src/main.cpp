#define _USE_MATH_DEFINES
#include "gridmap.h"
#include "odometry.h"
#include "planning.h"
#include "ublox_reader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char *argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <gps_data_file> <result_file>" << endl;
        return 1;
    }

    string gps_data = argv[1];
    string odom_commands = argv[2];

    auto result = readUbloxFile(gps_data);
    if(static_cast<int>(result.first.lat)==0 && static_cast<int>(result.first.lon)==0 && 
       static_cast<int>(result.second.lat)==0 && static_cast<int>(result.second.lon)==0) {
        cout<<"Error: Invalid GPS Coordinates"<<endl;
        return 1;
    }

    cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon << endl;
    cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon << endl;

    GPS origin = {result.first.lat, result.first.lon};
    double cellsize = 1.0;
    int rows = 10, cols = 10;
    Gridmapper grid(origin, cellsize, rows, cols);

    pair<int,int> start = grid.gpstogrid(result.first);
    pair<int,int> goal = grid.gpstogrid(result.second);

    cout << "Start (grid) -> (" << start.first << "," << start.second << ")" << endl;
    cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")" << endl;

    Planner planner(grid.getGrid());
    auto path = planner.pathplanning(start, goal);

    cout << "Planned Path:" << endl;
    for (auto &p : path) cout << "(" << p.first << "," << p.second << ") ";
    cout << endl;

    cout << "\nOdometry Commands" << endl;
    double wheel_radius = 0.05;
    double rpm = 120;
    Odometry odo(wheel_radius, rpm);
    MotionCommand commands = odo.computeCommands(path);

    ofstream result_file(odom_commands);
    if (!result_file.is_open()) {
        cerr << "Error: cannot open file " << odom_commands << endl;
        return 1;
    }
    result_file << commands.time_sec << endl;
    result_file << commands.angle_deg << endl;
    return 0;
}
