#include "car.h"

Car::Car(){
  x = 0.0;
  y = 0.0;
  s = 0.0;
  d = 0.0;
  speed = 0.0;
}


//-----------------------------------------------------

Ego::Ego(int id_){
  id = id_;
  yaw_deg = 0.0;
  yaw_rad = 0.0;
  speed = 0.0;
}

void Ego::update_current_status(double x_, double y_, double s_, double d_, double yaw_deg_, double speed_, vector<double> previous_path_x_, vector<double> previous_path_y_){
  x = x_;
  y = y_;
  s = s_;
  d = d_;
  yaw_deg = yaw_deg_;
  yaw_rad = deg2rad(yaw_deg_);
  speed = speed_;

  previous_size = previous_path_x_.size();
  cout << previous_size << endl;
  if(previous_size){
    previous_path.clear();
    previous_path.push_back(previous_path_x_);
    previous_path.push_back(previous_path_y_);
  }
  else{
    previous_path.clear();
  }
  previous_size = previous_path_x_.size();
}


std::ostream& operator<<(std::ostream& os, const Ego &car){
  os << "The values of car are:" << endl;
  os << "x: " << car.x << endl;
  os << "y: " << car.y << endl;
  os << "s: " << car.s << endl;
  os << "d: " << car.d << endl;
  os << "yaw_deg: " << car.yaw_deg << endl;
  os << "yaw_rad: " << car.yaw_rad << endl;
  os << "speed: " << car.speed << endl;
  for (int i = 0; i < car.previous_size; i++){
    os << "Step " << i+1 << ": x->  " << car.previous_path[0][i] << ", y-> " << car.previous_path[1][i] << endl;
  }
  
  os << "--------------------------" << endl;
  return os;
}


//--------------------------------------------------------

Other::Other(vector<double> sensor_fusion_) {
  id = sensor_fusion_[0];
  x = sensor_fusion_[1];
  y = sensor_fusion_[2];
  s = sensor_fusion_[4];
  d = sensor_fusion_[5];
  speed = sqrt(pow(sensor_fusion_[3],2) + pow(sensor_fusion_[4],2));
}

void Other::update(vector<double> sensor_fusion_){
  x = sensor_fusion_[1];
  y = sensor_fusion_[2];
  s = sensor_fusion_[4];
  d = sensor_fusion_[5];
  speed = sqrt(pow(sensor_fusion_[3],2) + pow(sensor_fusion_[4],2));
}

std::ostream& operator<<(std::ostream& os, const Other &car){
  os << "The values of car " << car.id << "  are:" << endl;
  os << "x: " << car.x << endl;
  os << "y: " << car.y << endl;
  os << "s: " << car.s << endl;
  os << "d: " << car.d << endl;
  os << "speed: " << car.speed << endl;
  
  os << "--------------------------" << endl;
  return os;
}
