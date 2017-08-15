#include "behavioral_planner.h"

Behavioral_planner::Behavioral_planner(Ego ego_car_, vector<vector<double>> sensor_fusion_) {
  ego_car = ego_car_;
  for(int i = 0; i < sensor_fusion_.size(); i++){
    Other car(sensor_fusion_[i]);
    other_cars.push_back(car);
    other_cars_tracker.push_back(car.id);
  }
  //cout << other_cars[3].s << endl;
}
