#ifndef BEHAVIORAL_PLANNER_H
#define BEHAVIORAL_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

class Behavioral_planner{
  private:
    Car ego_car;
    vector<vector<double>> sensor_fusion;
    string state;
    int optimal_lane;
    int target_lane;
    double target_speed;
  public:
    explicit Behavioral_planner();
    void plan(Ego ego_car_, vector<vector<double>> sensor_fusion_);
    
    void update_cars();
    int determine_optimal_lane();
    //This will be the final state machine which checks whether the path to the optimal lane is free
    string determine_optimal_state();
    int determine_target_lane();
    double determine_target_speed();
    int get_target_lane();
    double get_target_speed();
};

#endif
