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

  public:
    struct Planned{
      int lane;
      double speed;
    };

    explicit Behavioral_planner(Ego ego_car_, vector<vector<double>> sensor_fusion_);
    Planned plan();
    void update_cars();
    void determine_lane_costs();
    void determine_target_lane();
    void determine_target_speed();

    friend class Ego;
    friend class Other;
};

#endif
