#include "behavioral_planner.h"

Behavioral_planner::Behavioral_planner(Ego ego_car_, vector<vector<double>> sensor_fusion_) {
  // Maximum speed of the car in MPH
  maximum_speed = 49.5;
  // Time of prediction in the future
  dt = {1.0};
  ego_car = ego_car_;
  for(int i = 0; i < sensor_fusion_.size(); i++){
    Other car(sensor_fusion_[i]);
    other_cars.push_back(car);
    other_cars_tracker.push_back(car.id);
  }
  //cout << other_cars[3].s << endl;
}

Behavioral_planner::Planned Behavioral_planner::plan(){
  //cout << "1 :" << other_cars[2].s << endl;
  update_cars();
  //cout << "2 :" << other_cars[2].s_predictions[dt[0]] << endl;
  cout << other_cars[1];
  /*
   * BEGIN: Simple return values
   */
  Planned planned;
  planned.lane = 2;
  planned.speed = maximum_speed;
  /*
   * END: Simple return values
   */
  return planned;
}

void Behavioral_planner::update_cars(){
  for(int i = 0; i < dt.size(); i++){
    ego_car.predict(dt[i]);
    for(int j = 0; j < other_cars.size(); j++){
      other_cars[j].predict(dt[i]);
    }
  }
}
