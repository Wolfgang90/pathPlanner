#ifndef BEHAVIORAL_PLANNER_H
#define BEHAVIORAL_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
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
    vector<double> dt;
    vector<double> dt_weights;
    string state;
    int optimal_lane;
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
    map<int,double> determine_lane_costs();
    int determine_optimal_lane();
    //This will be the final state machine which checks whether the path to the optimal lane is free
    string determine_optimal_state();
    int determine_target_lane();
    double determine_target_speed();
    int get_target_lane();
    double get_target_speed();

    friend class Ego;
    friend class Other;
};

#endif
