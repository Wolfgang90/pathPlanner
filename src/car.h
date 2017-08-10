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
    int id;
    double s;
    double s_dot;
    double s_dotdot;
    double d;
    double d_dot;
    double d_dotdot;

    Car();
};

class Ego : public Car {
  public:
    double x;
    double y;
    double yaw_deg;
    double yaw_rad;
    double speed;
    vector<vector<double>> previous_path;
    int previous_size;

    Ego(int id_);
    void update_current_status(double x_, double y_, double s_, double d_, double yaw_deg_, double speed_, vector<double> previous_path_x_, vector<double> previous_path_y_);
    friend std::ostream& operator<<(std::ostream& os, const Ego &car);

};

#endif
