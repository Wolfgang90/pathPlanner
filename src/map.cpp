#include "map.h"

Map::Map(string map_file_){
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x_cur;
  	double y_cur;
  	float s_cur;
  	float dx_cur;
  	float dy_cur;
  	iss >> x_cur;
  	iss >> y_cur;
  	iss >> s_cur;
  	iss >> dx_cur;
  	iss >> dy_cur;
  	x.push_back(x_cur);
  	y.push_back(y_cur);
  	s.push_back(s_cur);
  	dx.push_back(dx_cur);
  	dy.push_back(dy_cur);
  }
  n = x.size();
}


void Map::print_map_values(){
  cout << "Current map values: " << endl;
  for (int i = 0; i < n; i++){
    cout << "Waypoint " << i << ": " << x[i] << ", " << y[i] << ", " << s[i] << ", " << dx[i] << ", " << dy[i] << endl;
  }
}
