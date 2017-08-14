#ifndef PREDICTION_H
#define PREDICTION_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "car.h"

using namespace std;

class Prediction{
  public:
    friend class Ego;
    friend class Other;
    //Ego& ego;
    //Other& other;
    Prediction(Ego& ego, double dt);
    Prediction(Other& other, double dt);
};

#endif
