#include "car.h"

Car::Car(){
  x = 0.0;
  y = 0.0;
  s = 0.0;
  d = 0.0;
  yaw = 0.0;
  speed = 0.0;
}


void Car::update_current_status(double x_, double y_, double s_, double d_, double yaw_, double speed_, vector<double> previous_path_x_, vector<double> previous_path_y_){
  x = x_;
  y = y_;
  s = s_;
  d = d_;
  yaw = yaw_;
  speed = speed_;

  vector<vector<double>> previous_path = 
  {
    previous_path_x_,
    previous_path_y_
  };

  previous_path_points = previous_path_x_.size();
}

/*
void Car::update_previous_path(vector<vector<double>> previous_path_){
  previous_path = previous_path_
}
*/

std::ostream& operator<<(std::ostream& os, const Car &car){
  os << "The values of car are:" << endl;
  os << "x: " << car.x << endl;
  os << "y: " << car.y << endl;
  os << "s: " << car.s << endl;
  os << "d: " << car.d << endl;
  os << "yaw: " << car.yaw << endl;
  os << "speed: " << car.speed << endl;
  for (int i = 0; i < car.previous_path_points; i++){
    os << "Step " << i+1 << ": x->  " << car.previous_path[0][i] << ", y-> " << car.previous_path[1][i] << endl;
  }
  os << "--------------------------" << endl;
  return os;
}
