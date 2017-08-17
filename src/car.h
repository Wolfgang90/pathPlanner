#ifndef CAR_H
#define CAR_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <math.h>
#include "helper.h"

using namespace std;

class Car {
  protected:
    void calculate_lane();

  public:
    int id;
    double x;
    double y;
    double s;
    map<double,double> s_predictions;
    double d;
    double speed;
    int lane;
    double max_s;
    string lane_status;

    Car();

    void check_lane_change();
};

class Ego : public Car {
  public:
    double yaw_deg;
    double yaw_rad;
    vector<vector<double>> previous_path;
    int previous_size;

    Ego();
    void update(double x_, double y_, double s_, double d_, double yaw_deg_, double speed_, vector<double> previous_path_x_, vector<double> previous_path_y_);
    void predict(double dt);
    friend std::ostream& operator<<(std::ostream& os, const Ego &car);
};

class Other: public Car {
  public:
    Other(vector<double> sensor_fusion_);
    void update(vector<double> sensor_fusion_);
    void predict(double dt);
    friend std::ostream& operator<<(std::ostream& os, const Other &car);
};

#endif
