#include "behavioral_planner.h"


Behavioral_planner::Behavioral_planner(Ego &ego_car_, vector<vector<double>> sensor_fusion_) {
  max_s = 6945.554;
  // Maximum speed of the car in MPH
  maximum_speed = 49.5;
  // Number of lanes
  num_lanes = 3;
  // Buffer value in front and behind of a car position not to be occupied by the own car
  car_buffer = 20.0;
  // Time of prediction in the future
  dt = {1.0,2.0,3.0};
  // Weights for each prediction in the future
  dt_weights = {0.7,0.2,0.1};

  map<int,double> lane_cost{{0,0.0},{1,0.0},{2,0.0}};

  ego_car = ego_car_;

  for(int i = 0; i < sensor_fusion_.size(); i++){
    Other car(sensor_fusion_[i]);
    other_cars.push_back(car);
  }
}


Behavioral_planner::Planned Behavioral_planner::plan(int previous_target_lane_){

  previous_target_lane = previous_target_lane_;

  update_cars();

  determine_lane_costs();

  Planned planned;
  determine_target_lane();
  planned.lane = target_lane;
  determine_target_speed();
  planned.speed = target_speed;

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


double Behavioral_planner::handle_lap_change(double ego_s, double other_s){
  // If the other car is already in the next lap
  if(other_s < (ego_s - 1000)){
    other_s = other_s +  max_s;

  //If the other car is still in the previous lap
  } else if(other_s > (ego_s + 1000)){
    other_s = other_s - max_s;
  }
  return other_s;
}


void Behavioral_planner::determine_lane_costs(){

  // Add costs for future states
  for(int i = 0; i < dt.size(); i++){
    
    double ego_car_s = ego_car.s_predictions[dt[i]];
    // Safety space in which no other car should be
    double safety_begin = ego_car_s - car_buffer;
    double safety_end = ego_car_s + car_buffer;

    // Map to get closest cars in front for each lane; lane -> distance from ego to closest car in front
    map<int,double> car_in_front{{0,max_s},{1,max_s},{2,max_s}};
    // Speed gap between car in front and maximum speed
    map<int,double> speedgap_in_front{{0,maximum_speed},{1,maximum_speed},{2,maximum_speed}};

    for (int j = 0; j < other_cars.size(); j++){
      // Extract required parameters for simplification
      double other_car_s = other_cars[j].s_predictions[dt[i]];
      int lane = other_cars[j].lane;

      other_car_s = handle_lap_change(ego_car_s, other_car_s);

      // Detremine car in front and speed gap for each lane
      if(other_car_s > ego_car_s){
        double gap = other_car_s - ego_car_s;
        if(gap < car_in_front[lane]){
          car_in_front[lane] = gap;
          speedgap_in_front[lane] = maximum_speed - other_cars[j].speed;
        }
      }

      //Add costs for car in safety space
      if(other_car_s > safety_begin && other_car_s < safety_end && ego_car.lane != lane){
        lane_cost[lane] += 1000;
      }
    }

    // Add costs per lane for car in front
    for(int k = 0; k < num_lanes; k++){
      lane_cost[k] = lane_cost[k] + dt_weights[i] * speedgap_in_front[k] * 10 / car_in_front[k];
    }
  }

  // Add cost for distance to current lane
  int current_lane = ego_car.lane; 
  for(int k = 0; k < num_lanes; k++){
    lane_cost[k] = lane_cost[k] + abs(k-current_lane) * 0.2;
  }

  // Add cost for current status
  // Add costs for lane change
  cur_id.assign(3,999);
  cur_delta_s.assign(3,max_s);
  cur_delta_speed.assign(3, maximum_speed);
  cur_delta_d.assign(3, 0.0);
  for(int i = 0; i < other_cars.size(); i++){
    int lane_other = other_cars[i].lane;
    double ego_s = ego_car.s;
    double other_s = other_cars[i].s;
    other_s = handle_lap_change(ego_s, other_s);
    if(other_s > ego_s){
      double gap = other_s - ego_s;
      if(gap < cur_delta_s[lane_other]){
        cur_id[lane_other] = other_cars[i].id;
        cur_delta_s[lane_other] = gap;
        cur_delta_speed[lane_other] = other_cars[i].speed - ego_car.speed;
        cur_delta_d[lane_other] = other_cars[i].d - (2.0 + 4.0 * lane_other);
      }
    }
  }

  for(int i = 0; i < num_lanes; i++){
    lane_cost[i] = lane_cost[i] + abs(cur_delta_speed[i]) * 10 / cur_delta_s[i];
    //In case the car is in a lane change to the right
    
    if(cur_delta_d[i] > 0.5 && (i+1) < num_lanes-1){
      lane_cost[i+1] = lane_cost[i+1] + 4 * abs(cur_delta_speed[i]) * 10 / cur_delta_s[i];
    }
    //In case the car is in a lane change to the right
    if(cur_delta_d[i] < -0.5 && (i-1) >= 0){
      lane_cost[i-1] = lane_cost[i-1] + 4 * abs(cur_delta_speed[i]) * 10 / cur_delta_s[i];
    }
  }
}


void Behavioral_planner::determine_target_lane(){
  // If the car seems to be in a lane change don't change the target lane identified in the last round
  ego_car.check_lane_change();
  if(ego_car.lane_status == "lane_change"){
    target_lane = previous_target_lane;
    return;
  }

  // Get target lane based on lowest cost value
  double cost_tmp = 100000;
  
  for (auto const& lc: lane_cost){
    if(lc.second < cost_tmp){
      cost_tmp = lc.second;
      target_lane = lc.first;
    }
  }

  // Handle issues of a blocked lane in-between
  int current_lane = ego_car.lane;
  // If target lane to the right
  if (target_lane > current_lane + 1){
    for(int i = current_lane + 1; i < target_lane; i++) {
      if(lane_cost[i]>100.0){
        target_lane = i-1;
        return;
      }
    }
  // If target lane to the left
  } else if(target_lane < current_lane-1){
    for(int i = current_lane - 1; i > target_lane; i--) {
      if(lane_cost[i]>100.0){
        target_lane = i+1;
        return;
      }
    }
  }
}


void Behavioral_planner::determine_target_speed(){

  double delta_s = cur_delta_s[ego_car.lane];
  double delta_speed = cur_delta_speed[ego_car.lane];

  double thresh_low = 15.0;
  double thresh_high = 40.0;
  if(delta_s > thresh_high){
    target_speed = maximum_speed;
  }
  else if(delta_s > thresh_low){
    target_speed = (ego_car.speed + delta_speed) + abs(delta_speed) * ((delta_speed-thresh_low)/(thresh_high-thresh_low));
  }
  else {
    target_speed = ego_car.speed + delta_speed;
  }
}
