#include "car.h"

Car::Car(){
  x = 0.0;
  y = 0.0;
  s = 0.0;
  d = 0.0;
  yaw = 0.0;
  speed = 0.0;
}


void Car::update_current_status(double x_, double y_, double s_, double d_, double yaw_, double speed_){
  x = x_;
  y = y_;
  s = s_;
  d = d_;
  yaw = yaw_;
  speed = speed_;
}

/*
void Car::update_previous_path(vector<vector<double>> previous_path_){
  previous_path = previous_path_
}
*/

void Car::print_car_values(){
  cout << "The values of car are:" << endl;
  cout << "x: " << x << endl;
  cout << "y: " << y << endl;
  cout << "s: " << s << endl;
  cout << "d: " << d << endl;
  cout << "yaw: " << yaw << endl;
  cout << "speed: " << speed << endl;
  cout << "--------------------------" << endl;
}
