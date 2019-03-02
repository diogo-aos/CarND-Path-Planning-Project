#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "spline.h"

#include "helpers.h"
#include "json.hpp"

/* * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *         MY CONSTANTS
 *
 * * * * * * * * * * * * * * * * * * * * * * * * */


#define DEBUG 0

/* * * * * * * * * * * * * * * * * * * * * * * * *
 *                                               *
 *                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * */

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;


/* * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *         MY VARIABLES
 *
 * * * * * * * * * * * * * * * * * * * * * * * * */

double TO_MPH = 2.23694;  // m.s to mph
double TO_MS = 0.44704;   // mph to m.s

double GOAL_SPEED = 49.5; //ms = 49.8mph
double TARGET_SPEED = 49.5; //ms

double SLOW_ACC = 0.22369;  // 5m.s^2
double FAST_ACC = 0.4250186;  // 9.5m.s^2

double ref_speed = 0; //mph
int lane = 1;

double acc;

class Car {
public:
  int id;

  double x;
  double y;
  double yaw;

  double vx;
  double vy;

  double v;     // m.s
  double v_mph; // mph

  double s;
  double d;

  int lane;

  
  double update_v(double new_vx, double new_vy){
    vx = new_vx;
    vy = new_vy;
    // compute speed
    v = sqrt(vx*vx + vy*vy);
    v_mph = v * TO_MPH;
    return v;
  }
  
  double update_loc(double new_d, double new_s){
    s = new_s;
    d = new_d;

    // compute lane
    if (d>0 && d<4)
      lane = 0;
    else if(d>4 && d<8)
      lane = 1;
    else if(d>8 && d<12)
      lane = 2;
    return d;
  }
  
};


// keep track of other cars
vector <Car> cars;
Car EC;

/* * * * * * * * * * * * * * * * * * * * * * * * *
 *                                               *
 *                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * */


void log_print(char * str, int level){
  /*
   * 0 - DEBUG ALL
   * 1 - DEBUG DEV
   */
  if (level >= DEBUG)
    std::cout << str << std::endl;
}

double poly5_x(vector<double> &coefs, double x){

  double res=coefs[0];
  double exp = x;
//  std::cout << "poly5x  start"  << std::endl;
  for (int i=1; i<6; i++){

    res += coefs[i] * exp;
//  std::cout << "poly5x  " << res << " = " << coefs[i] << " * " << exp << std::endl;
    exp *= x;
  }
return res;
}

vector<double> poly5_vx(vector<double> &coefs, int n_steps, double inc){
  vector<double> out;
  for(double i=1; i<=n_steps; i++)
    out.push_back(poly5_x(coefs, i*inc));
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
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
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



          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
           *      
	   *              START OF CODE
           *   
           * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */




	  int n_wps = 50;
	  int prev_size = previous_path_x.size();

	  // print car state
	  for (int i=0; i<30; i++)
	    std::cout << "- ";
	  std::cout << std::endl;

	  std::cout << "x:" << car_x;
	  std::cout << "\t y:" << car_y;
	  std::cout << "\t yaw:" << car_yaw;
	  std::cout << std::endl;

	  std::cout << "s:" << car_s;
	  std::cout << "\t d:" << car_d;
	  std::cout << "\t speed:" << car_speed;
	  std::cout << "\ttarget speed:" << TARGET_SPEED;
	  std::cout << std::endl;

	  std::cout << "lane:" << lane;
	  std::cout << "\tref speed:" << ref_speed;
	  std::cout << std::endl;

	  std::cout << "Previous path size: " << prev_size << "\t consumed " << n_wps - previous_path_x.size() << " waypoints" << std::endl;
	  std::cout << "consumed " << n_wps - previous_path_x.size() << " waypoints" << std::endl;


#if DEBUG == 1
	  // print previous path
	  std::cout << "previous path:" << std::endl;
	  for(int i=0; i<previous_path_x.size();i++)
	    std::cout << "(" << previous_path_x[i] << "," << previous_path_y[i] << "),";
	  std::cout << std::endl;
#endif


	  if(prev_size > 0){
	    car_s = end_path_s;
	  }


	  TARGET_SPEED = GOAL_SPEED;
          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
           *      
	   *              CHECK OTHER CARS
           *   
           * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	  std::cout << "PHASE | Checking other cars..." << std::endl;


	  bool too_close = false;
	  double lane_forward[3];
	  double lane_backward[3];

	  vector <double> check_s_t;


	  double my_lane_center = lane * 4 + 2;


	  /*
	   * update our internal library of cars
	   */
	  for (int i=0; i < sensor_fusion.size(); i++){
	    int id = sensor_fusion[i][0];
	    int cars_i=cars.size();  // by default assume it's a new car that will be added

	    // look for this car's ID in our car library
	    for (int j=0; j<cars.size(); j++){
	      if (cars[j].id == id){  // found car id in library!
		cars_i = j;
		break;
	      }
	    }

	    // if it's not in library, create new car
	    if (cars_i == cars.size()){
	      Car C;
	      C.id = id;
	      cars.push_back(C);
	    }

	    // update this car's values from sensor fusion
	    cars[cars_i].id = sensor_fusion[i][0];
	    cars[cars_i].update_loc(sensor_fusion[i][6], sensor_fusion[i][5]);
	    cars[cars_i].update_v(sensor_fusion[i][3], sensor_fusion[i][4]);
	  }

	  //find ref_v to use
	  for (int i=0; i < cars.size(); i++){
	    // print car info
	    std::cout << "Car " << cars[i].id;
	    std::cout << "\tv:" << cars[i].v_mph;
	    std::cout << "\ts:" << cars[i].s;
	    std::cout << "\td:" << cars[i].d;
	    std::cout << std::endl;

	    // transform check_s so car_s is reference

	    //car is in my lane
	    if ( (cars[i].d > my_lane_center - 2) && (cars[i].d < my_lane_center + 2) ){
    
	      // if using previous points can project s value out
	      double check_s = cars[i].s;
	      check_s += ((double) prev_size * 0.02 * cars[i].v);  
	      
	      //check s values greater than mine and s gap
	      if ((check_s > car_s) && (check_s-car_s) < 30){
		std::cout << "Car " << cars[i].id << "triggered lane change" << std::endl;
		std::cout << "my_lane_center:" << my_lane_center << std::endl;
		//do some logic here, lower reference velocity so we don't crash 
		// into the car in front of us, ould also flag to try to change
		// lanes
		//ref_speed = check_speed; //mph
		too_close = true;
		TARGET_SPEED = cars[i].v_mph - FAST_ACC;
		if (lane > 0) {
		  lane = 0;
		}
	      }
	    } // end if car is in my lane
	  } // end for sensor fusion

	  if (ref_speed - TARGET_SPEED > FAST_ACC){
	    ref_speed -= FAST_ACC;
	  }
	  else if(ref_speed - TARGET_SPEED < - FAST_ACC){
	    ref_speed += FAST_ACC;
	  }

          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
           *      
	   *              DECISION MAKING
           *   
           * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */





          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
           *      
	   *              GENERATE TRAJECTORY
           *   
           * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	  std::cout << "PHASE | Starting trajectory generation..." << std::endl;

	  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	  // later we will interpolate these waypoints with a spline and fill it
	  // in with more points that control speed.
	  vector <double> ptsx;
	  vector <double> ptsy;


	  // reference x,y, yaw states
	  // either we will reference the starting point as where the car is or at
	  // the previous path's end point

	  double ref_x = car_x;
	  double ref_y = car_y;
	  double ref_yaw = deg2rad(car_yaw);

	  // if previous path's size is almost empty, use the car as starting reference
	  if (prev_size < 2){
	    // use two points that make the path tangent to the car,
	    // projecting bicycle model to the past
	    double prev_car_x = car_x - cos(car_yaw);
	    double prev_car_y = car_y + sin(car_yaw);

	    ptsx.push_back(prev_car_x);
	    ptsx.push_back(car_x);

	    ptsy.push_back(prev_car_y);
	    ptsy.push_back(car_y);

	  }
	  // use previous path's end point as starting reference
	  else {
	    // redefine reference state to be previous path's end point
	    ref_x = previous_path_x[prev_size-1];
	    ref_y = previous_path_y[prev_size-1];

	    double ref_x_prev = previous_path_x[prev_size-2];
	    double ref_y_prev = previous_path_y[prev_size-2];

	    ref_yaw = atan2(ref_y - ref_y_prev,
			    ref_x - ref_x_prev);  // arctan

	    // use two points that make the path tangent to the previous path's end point
	    ptsx.push_back(ref_x_prev);
	    ptsx.push_back(ref_x);

	    ptsy.push_back(ref_y_prev);
	    ptsy.push_back(ref_y);
  
	  }

	  // in Frenet add evenly 30m spaced points ahead of the the starting reference
	  for(int i=1; i<=3; i+=1){
	    double next_s = car_s + i*30;  // 30m steps
	    double next_d = 2 + 4 * lane;
	    vector <double> p = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    ptsx.push_back(p[0]);
	    ptsy.push_back(p[1]);
	  }

#if DEBUG == 1
	  std::cout << "ptsx ptsy raw" << ptsx.size() << std::endl;
	  std::cout << "ptsx size=" << ptsx.size() << std::endl;
	  std::cout << "ptsy size=" << ptsy.size() << std::endl;
	  for (int i=0; i<ptsx.size(); i++)
	    std::cout << "i " << i << " x = " << ptsx[i] << " y = " << ptsy[i] << std::endl;
#endif

	  for (int i = 0; i < ptsx.size(); i++){
  
	    //shift point to car reference
	    // i.e. shift point such that car (ref_x, ref_y) is now at origin
	    double shift_x = ptsx[i]-ref_x;
	    double shift_y = ptsy[i]-ref_y;

	    // rotate point so that ref_yaw is 0 degrees
	    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
	    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

	  }

#if DEBUG == 1
	  std::cout << "ptsx ptsy transformed" << ptsx.size() << std::endl;
	  std::cout << "ptsx size=" << ptsx.size() << std::endl;
	  std::cout << "ptsy size=" << ptsy.size() << std::endl;
	  for (int i=0; i<ptsx.size(); i++)
	    std::cout << "i " << i << " x = " << ptsx[i] << " y = " << ptsy[i] << std::endl;
#endif


	  // create spline
	  tk::spline s;

	  // set x,y points to the spline
	  s.set_points(ptsx, ptsy);


	  // use all unused points from previous path in next path
	  for(int i=0; i<prev_size; i++){
	    next_x_vals.push_back(previous_path_x[i]);
	    next_y_vals.push_back(previous_path_y[i]);
	  }

	  // calculate how to break up spline points so that we travel 
	  // at our desired reference velocity
	  double target_x = 30.0; // look at a 30m horizon
	  double target_y = s(target_x);
	  double target_dist = sqrt(target_x*target_x + target_y*target_y);

	  double x_add_on = 0;

	  // fill up the rest of our path planner after filling it with
	  // previous points, here we will always output n_wp (50) points
	  for (int i=0; i<=n_wps-prev_size; i++){
	    double N = target_dist / (0.02 * ref_speed / 2.24);  // 1 mph / 2.24 -> m/s
	    double x_point = x_add_on  + (target_x) / N;
	    double y_point = s(x_point);

	    x_add_on = x_point;

	    double x_ref = x_point;  // not to be confused with ref_x
	    double y_ref = y_point;



	    // undo rotation
	    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
	    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

	    // undo shift
	    x_point += ref_x;
	    y_point += ref_y;

	    next_x_vals.push_back(x_point);
	    next_y_vals.push_back(y_point);

	  }

#if DEBUG == 1
	  // print next path
	  std::cout << "next path:" << std::endl;
	  for(int i=0; i<next_x_vals.size();i++)
	    std::cout << "(" << next_x_vals[i] << "," << next_y_vals[i] << "),";
	  std::cout << std::endl;
#endif

	  // -------------------------------------------------------
	  //               END OF CODE
	  // -------------------------------------------------------

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
