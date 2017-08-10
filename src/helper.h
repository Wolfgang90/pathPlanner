#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <map>
#include "car.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

class Helper{
  public:
    double distance(double x1, double y1, double x2, double y2)
    {
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }
    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
    {
      int prev_wp = -1;

      while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
      {
        prev_wp++;
      }

      int wp2 = (prev_wp+1)%maps_x.size();

      double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
      // the x,y,s along the segment
      double seg_s = (s-maps_s[prev_wp]);

      double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
      double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

      double perp_heading = heading-pi()/2;
      double x = seg_x + d*cos(perp_heading);
      double y = seg_y + d*sin(perp_heading);

      return {x,y};

    }

};

class Ego;
class Other;
struct Status{
  Ego* ego_car;
  vector<Other> sensor_fusion;
};



#endif
