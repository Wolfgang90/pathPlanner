#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

class Map {
  private:
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> dx;
    vector<double> dy;
    double n;

  public:
    explicit Map(string map_file);
    void print_map_values();
};

#endif
