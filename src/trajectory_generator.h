#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "map.h"
#include "car.h"

using namespace std;

class Trajectory_generator{
  private:
    Map& track;
    double target_speed;
  public:
    explicit Trajectory_generator(Map& track_, double target_speed_);//: track(track_) {}
    vector<vector<double>> generate(Car ego_car_, int target_lane_);
};

#endif