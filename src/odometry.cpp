#define _USE_MATH_DEFINES
#include "odometry.h"
#include <cmath>
#include <vector>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (grid units per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  // Euclidean distance in grid units
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res = {0.0, 0.0}; // total angle_deg, total time_sec

  for (size_t i = 1; i < path.size(); ++i) {
      int x1 = path[i - 1].first;
      int y1 = path[i - 1].second;
      int x2 = path[i].first;
      int y2 = path[i].second;

      res.time_sec += distance(x1, y1, x2, y2) / linear_vel;  // time in seconds
      res.angle_deg += fabs(angle(x1, y1, x2, y2));           // angle in degrees
  }

  return res;
}

