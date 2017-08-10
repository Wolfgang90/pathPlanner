#ifndef PREDICTION_H
#define PREDICTION_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "helper.h"

using namespace std;

class Prediction{
  public:
    Prediction();
    //TODO: Create datatype to return the following:
    //map<int,Other> 
    //Ego ego_car;
    datatype get_prediction(Ego ego_car_, vector<vector<double>> sensor_fusion_, double dt);
};

#endif
