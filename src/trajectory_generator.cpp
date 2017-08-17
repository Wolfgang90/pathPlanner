#include "trajectory_generator.h"

Trajectory_generator::Trajectory_generator(Map& track_) : track(track_), target_speed(0.0), ref_vel(0.0), acc_factor(1.4), dec_factor(1.4) {};

vector<vector<double>> Trajectory_generator::generate(Ego ego_car_, int target_lane_, double target_speed_){

  target_speed = target_speed_;

  if(ref_vel > target_speed + dec_factor){
    ref_vel -= dec_factor;
  }
  else if(ref_vel < target_speed - acc_factor){
    ref_vel += acc_factor;
  }
  else{
    ref_vel = target_speed;
  }

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // They will be interpolated with a spline later

  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states
  // either the starting point is were the car is or at the previous paths end point
  
  double ref_x = ego_car_.x;
  double ref_y = ego_car_.y;
  double ref_yaw = ego_car_.yaw_rad;

  double ref_lane = ego_car_.lane;

  // if previous size is almost empty, use the car as starting reference
  if(ego_car_.previous_size < 2){
    // Use two points that make the path tangent to the car
    double prev_car_x = ego_car_.x - cos(ego_car_.yaw_deg);
    double prev_car_y = ego_car_.y - sin(ego_car_.yaw_deg);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ego_car_.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ego_car_.y);
  }
  // use the previous path's point as starting reference
  else {

    // Redefine reference state as previous path end point
    ref_x = ego_car_.previous_path[0][ego_car_.previous_size-1];
    ref_y = ego_car_.previous_path[1][ego_car_.previous_size-1];

    double ref_x_prev = ego_car_.previous_path[0][ego_car_.previous_size-2];
    double ref_y_prev = ego_car_.previous_path[1][ego_car_.previous_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    //Use two point that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference
  Helper h;
  vector<double> next_wp0 = h.getXY(ego_car_.s + 50, (2+4*target_lane_), track.s, track.x, track.y);
  vector<double> next_wp1 = h.getXY(ego_car_.s + 100, (2+4*target_lane_), track.s, track.x, track.y);
  vector<double> next_wp2 = h.getXY(ego_car_.s + 150, (2+4*target_lane_), track.s, track.x, track.y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  for(int i = 0; i < ptsx.size(); i++) {

    //shift car reference angle to 0 degrees to prevent spline from having multiple x-values
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  // create spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx,ptsy);

  // define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with all the previous path points from last time
  for(int i = 0; i < ego_car_.previous_size; i++){
    next_x_vals.push_back(ego_car_.previous_path[0][i]);
    next_y_vals.push_back(ego_car_.previous_path[1][i]);
  }

  // calculate how to break up spline point so that we travel at our desired reference velocity
  double target_x = 50;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
  
  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for (int i = 1; i <= 50-ego_car_.previous_size; i++){

    double N = (target_dist/(0.02*ref_vel/2.24));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);
  return next_vals;
}
