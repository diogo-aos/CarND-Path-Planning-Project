#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

double GOAL_SPEED = 50; //mph
double TARGET_SPEED = 50; //mph

double poly5_x(vector<double> &coefs, double x){
  double res=coefs[0];
  double exp=x;
  for (int i=1; i<6; i++){
    res += coefs[i] * x;
    x *= x;
  }
}

double poly5_vx(vector<double> &coefs, double duration, int n_steps){
  vector<double> out;
  double inc = duration / n_steps;
  for(int i=1; i<=n_steps; i++)
    out.push_back(poly5_x(coefs, i*inc);
  return out;

}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   double a0 = start[0],
          a1 = start[1],
          a2 = start[2] / 2;
          
   //A.X=B -> X=A-1.B
    MatrixXd A(3,3);
    MatrixXd B(3,1);
    A << pow(T, 3), pow(T, 4), pow(T, 5),
         3*pow(T, 2), 4*pow(T, 3), 5*pow(T,4),
         6*T, 12*pow(T, 2), 20*pow(T, 3);
    B << end[0] - (start[0] + start[1] * T + start[2] * pow(T, 2) / 2),
         end[1] - (start[1] + start[2] * T),
         end[2] - start[2];
         
    MatrixXd X = A.inverse() * B;

  return {a0, a1, a2, X.data()[0], X.data()[1], X.data()[2]};
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
	  
	  std::cout << "sensor_fusion" << std::endl;
	  for(int i=0; i<sensor_fusion.size(); i++){
	    std::cout << "id:" << sensor_fusion[i][0] << "\t s:" << sensor_fusion[i][5] << "\t d:" << sensor_fusion[i][6] << std::endl;
	  }
	  std::cout << "car_s:" << car_s << "\t car_d:" << car_d << std::endl;

	  int current_state = 0;
	  int next_state = 0;
	  switch(current_state){
	  case 0: { // ready
	    break;
	  }
	  case 1: { // keep lane
	    break;
	  }
	  case 2: { // prepare lane change R
	    break;
	  }
	  case 3: { // lane change R
	    break;
	  }
	  case 4: { // prepare lane change L
	    break;
	  }
	  case 5: { // lane change L
	    break;
	  }
	  }  //end of switch

	  std::cout << previous_path_x.size() << std::endl;

	  double dist_inc = (TARGET_SPEED * 1.6) * 1000 / 3600 / 50;
	  std::cout << "dist_inc=" << dist_inc << std::endl;
	  //dist_inc = 0.4;
	  double s, d;
	  vector <double> xy;
	  s = car_s;
	  d = car_d;
	  xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  next_x_vals.push_back(xy[0]);
	  next_y_vals.push_back(xy[1]);
	  for (int i=1; i<50; i++){
	    s = s + dist_inc;
	    d = d;
	    xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_x_vals.push_back(xy[0]);
	    next_y_vals.push_back(xy[1]);
	  }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
