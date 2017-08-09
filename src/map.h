#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

class Map {
  public:
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> dx;
    vector<double> dy;
    double n;

    explicit Map(string map_file);
    friend std::ostream& operator<<(std::ostream& os, const Map &map);
};

#endif
