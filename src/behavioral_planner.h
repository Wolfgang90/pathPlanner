#ifndef BEHAVIORAL_PLANNER_H
#define BEHAVIORAL_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include "car.h"

using namespace std;

class Behavioral_planner{
  private:
    double max_s;
    Ego ego_car;
    vector<Other> other_cars;
    double maximum_speed;
    int num_lanes;
    double car_buffer;
    vector<double> dt;
    vector<double> dt_weights;
    map<int,double> lane_cost;
    int target_lane;
    double target_speed;
    int previous_target_lane;
    vector<int> cur_id;
    vector<double> cur_delta_s;
    vector<double> cur_delta_speed;
    vector<double> cur_delta_d;

    double handle_lap_change(double ego_s, double other_s);

  public:
    struct Planned{
      int lane;
      double speed;
    };

    explicit Behavioral_planner(Ego& ego_car_, vector<vector<double>> sensor_fusion_);
    Planned plan(int previous_target_lane_);
    void update_cars();
    void determine_lane_costs();
    void determine_target_lane();
    void determine_target_speed();

    friend class Ego;
    friend class Other;
};

#endif
