#include "car.h"

Car::Car(){
  x = 0.0;
  y = 0.0;
  s = 0.0;
  d = 0.0;
  speed = 0.0;
  lane = 0;
  // The max s value before wrapping around the track back to 0
  max_s = 6945.554;
  lane_status = "in_lane";
}


void Car::calculate_lane(){
  lane = int(floor(d / 4.0));
}


void Car::check_lane_change(){
  if(d < 1.75 || d > 10.25){
    lane_status = "out_of_lane";
  } else if ( d > 1.75 && d < 2.25 || d > 5.75 && d < 6.25 || d > 9.75 && d < 10.25) {
    lane_status = "in_lane";
  } else{
    lane_status = "lane_change";
  }
}

//-----------------------------------------------------

Ego::Ego(){
  id = 1000;
  yaw_deg = 0.0;
  yaw_rad = 0.0;
  speed = 0.0;
}


void Ego::update(double x_, double y_, double s_, double d_, double yaw_deg_, double speed_, vector<double> previous_path_x_, vector<double> previous_path_y_){
  x = x_;
  y = y_;
  s = s_;
  d = d_;
  yaw_deg = yaw_deg_;
  yaw_rad = deg2rad(yaw_deg_);
  speed = speed_;

  previous_size = previous_path_x_.size();
  if(previous_size){
    previous_path.clear();
    previous_path.push_back(previous_path_x_);
    previous_path.push_back(previous_path_y_);
  }
  else{
    previous_path.clear();
  }
  previous_size = previous_path_x_.size();
  calculate_lane();
}


void Ego::predict(double dt){
  s_predictions[dt] = fmod(s + speed/2.24 * dt,max_s);
}


std::ostream& operator<<(std::ostream& os, const Ego &car){
  os << "The values of car are:" << endl;
  os << "x: " << car.x << endl;
  os << "y: " << car.y << endl;
  os << "s: " << car.s << endl;
  os << "s_predictions: " << endl;
  for (auto const& s_pred: car.s_predictions){
    os << s_pred.first << ":" << s_pred.second << endl;
  }
  os << "d: " << car.d << endl;
  os << "yaw_deg: " << car.yaw_deg << endl;
  os << "yaw_rad: " << car.yaw_rad << endl;
  os << "lane: " << car.lane << endl;
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
  s = sensor_fusion_[5];
  d = sensor_fusion_[6];
  speed = sqrt(pow(sensor_fusion_[3],2) + pow(sensor_fusion_[4],2));
  calculate_lane();
}


void Other::update(vector<double> sensor_fusion_){
  x = sensor_fusion_[1];
  y = sensor_fusion_[2];
  s = sensor_fusion_[5];
  d = sensor_fusion_[6];
  speed = sqrt(pow(sensor_fusion_[3],2) + pow(sensor_fusion_[4],2));
  calculate_lane();
}


void Other::predict(double dt){
  s_predictions[dt] = fmod(s + speed/2.24 * dt,max_s);
}


std::ostream& operator<<(std::ostream& os, const Other &car){
  os << "The values of car " << car.id << "  are:" << endl;
  os << "x: " << car.x << endl;
  os << "y: " << car.y << endl;
  os << "s: " << car.s << endl;
  os << "s_predictions: " << endl;
  for (auto const& s_pred: car.s_predictions){
    os << s_pred.first << ":" << s_pred.second << endl;
  }
  os << "d: " << car.d << endl;
  os << "speed: " << car.speed << endl;
  os << "lane: " << car.lane << endl;
  
  os << "--------------------------" << endl;
  return os;
}
