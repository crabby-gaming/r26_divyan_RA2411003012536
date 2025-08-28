#define _USE_MATH_DEFINES
#include "Odometry.h"
#include <cmath>
#include <vector>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
    // Linear velocity (grid units per second)
    double rps = rpm / 60.0;
    linear_vel = 2 * M_PI * radius * rps;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
    MotionCommand res = {0.0, 0.0};
    if (path.size() < 2) return res;

    int sx = path.front().first;
    int sy = path.front().second;
    int gx = path.back().first;
    int gy = path.back().second;

    double dx = gx - sx;
    double dy = gy - sy;

    // Euclidean straight-line distance
    double dist = sqrt(dx * dx + dy * dy);

    // Heading in degrees, normalized to [0,360)
    double angle = atan2(dy, dx) * 180.0 / M_PI;
    if (angle < 0) angle += 360.0;

    // Convert distance to time
    res.time_sec = dist / linear_vel;
    res.angle_deg = angle;

    return res;
}



