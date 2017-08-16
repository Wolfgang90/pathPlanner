#include "behavioral_planner.h"

Behavioral_planner::Behavioral_planner(Ego ego_car_, vector<vector<double>> sensor_fusion_) {
  max_s = 6945.554;
  // Maximum speed of the car in MPH
  maximum_speed = 49.5;
  // Number of lanes
  num_lanes = 3;
  // Time of prediction in the future
  dt = {1.0,2.0,3.0};
  // Weights for each prediction in the future
  dt_weights = {0.7,0.2,0.1};
  ego_car = ego_car_;
  for(int i = 0; i < sensor_fusion_.size(); i++){
    Other car(sensor_fusion_[i]);
    other_cars.push_back(car);
  }
  //cout << other_cars[3].s << endl;
}

Behavioral_planner::Planned Behavioral_planner::plan(){
  //cout << "1 :" << other_cars[2].s << endl;
  update_cars();
  //cout << "ego_car.s_predictions[0]: " << ego_car.s_predictions[0] << endl;
  //cout << "ego_car.s_predictions[1]: " << ego_car.s_predictions[1] << endl;
  //cout << "ego_car.s_predictions[2]: " << ego_car.s_predictions[2] << endl;
  //cout << "2 :" << other_cars[2].s_predictions[dt[0]] << endl;
  //cout << other_cars[1];
  map<int,double> lane_cost = determine_lane_costs();
  /*
   * BEGIN: Simple return values
   */
  Planned planned;
  planned.lane = 1;
  planned.speed = maximum_speed;
  /*
   * END: Simple return values
   */
  return planned;
}

void Behavioral_planner::update_cars(){
  for(int i = 0; i < dt.size(); i++){
    //cout << "Car predict: " << i << endl;
    ego_car.predict(dt[i]);
    for(int j = 0; j < other_cars.size(); j++){
      other_cars[j].predict(dt[i]);
    }
  }
}

map<int,double> Behavioral_planner::determine_lane_costs(){
  map<int,double> lane_cost{{0,0.0},{1,0.0},{2,0.0}};
  for(int i = 0; i < dt.size(); i++){
    //cout << "test 1" << endl;
    double ego_car_s = ego_car.s_predictions[dt[i]];
    //cout << ego_car;

    // Map to get closest cars in front for each lane; lane -> distance from ego to closest car in front
    map<int,double> car_in_front{{0,max_s},{1,max_s},{2,max_s}};
    // Speed gap between car in front and maximum speed
    map<int,double> speedgap_in_front{{0,maximum_speed},{1,maximum_speed},{2,maximum_speed}};
    for (int j = 0; j < other_cars.size(); j++){
      //cout << "test 2" << endl;
      // Extract required parameters for simplification
      double other_car_s = other_cars[j].s_predictions[dt[i]];
      int lane = other_cars[j].lane;

      //cout << other_cars[j];
      //cout << "s here: " << other_car_s << endl;

      // Handle lap change
      // If the other car is already in the next lap
      if(other_car_s < (ego_car_s - 1000)){
        other_car_s = other_car_s +  max_s;
        //cout << "already in next lap" << endl;
      //If the other car is still in the previous lap
      } else if(other_car_s > (ego_car_s + 1000)){
        //cout << "still in previous lap" << endl;
        other_car_s = other_car_s - max_s;
      }


      //cout << "ego_car_s: " << ego_car_s << endl;
      //cout << "other_car_s: " << other_car_s << endl;
      if(other_car_s > ego_car_s){
        //cout << "reduction for " << j << endl;
        double gap = other_car_s - ego_car_s;
        if(gap < car_in_front[lane]){
          car_in_front[lane] = gap;
          speedgap_in_front[lane] = maximum_speed - other_cars[j].speed;
        }
      }
      //cout << "Ego: " << ego_car_s << endl;
      //cout << "Other: " << other_car_s << endl;

      // 1 Get costs for cars in front
      // For each lane -> Identify closest car in front
      // Create a function of distance to car in front and speed on the lane

      // 2 Get costs for cars blocking lane change
      // Get costs for lanes on the right
      // Get costs for lanes on the left
    }

  for (auto const& lane_label: lane_cost){
    //cout << "lane_label.first: " << lane_label.first << endl;
    lane_cost[lane_label.first] += dt_weights[i] * speedgap_in_front[lane_label.first] / car_in_front[lane_label.first];
  }

  //cout << "Lane 0 " << car_in_front[0] << endl;
  //cout << "Lane 1 " << car_in_front[1] << endl;
  //cout << "Lane 2 " << car_in_front[2] << endl;
  }

  cout << "Lane 0 cost: " << lane_cost[0] << endl;
  cout << "Lane 1 cost: " << lane_cost[1] << endl;
  cout << "Lane 2 cost: " << lane_cost[2] << endl;

  /*
   * BEGIN: Simple lane_cost
   */
   //lane_cost = {{0,10.0}, {1,11.0}, {2,12.0}};
  /*
   * END: Simple lane_cost
   */
  return lane_cost;
}
