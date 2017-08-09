#ifndef CAR_H
#define CAR_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "helper.h"


using namespace std;

class Car {
  public:
    double x;
    double y;
    double s;
    double d;
    double yaw_deg;
    double yaw_rad;
    double speed;
    vector<vector<double>> previous_path;
    int previous_path_points;

    Car();
    void update_current_status(double x_, double y_, double s_, double d_, double yaw_deg_, double speed_, vector<double> previous_path_x_, vector<double> previous_path_y_);
//    void update_previous_path(vector<vector<double>> previous_path_); 
    friend std::ostream& operator<<(std::ostream& os, const Car &car);
};

#endif
