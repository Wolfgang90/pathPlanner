#include "trajectory_generator.h"

Trajectory_generator::Trajectory_generator(Map& track_, double target_speed_) : track(track_), target_speed(target_speed_) {};

vector<vector<double>> Trajectory_generator::generate(Car ego_car_, int target_lane_){

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // They will be interpolated with a spline later

  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states
  // either the starting point is were the car is or at the previous paths end point
  
  double ref_x = ego_car_.x;
  double ref_y = ego_car_.y;
  double ref_yaw = ego_car_.yaw_rad;

  // if previous size is almost empty, use the car as starting reference
}
