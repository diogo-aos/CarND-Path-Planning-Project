int lane = 1;
double ref_speed = 49.5;  // mph


prev_size = previous_path_x.size();

if(prev_size > 0){
  car_s = end_path_s;
}

bool too_close = false;

//find ref_v to use
for (int i=0; i < sensor_fusion.size(); i++){

  //car is in my lane
  double d = sensor_fusion[i][6];
  double my_lane_center = 2 * 4 * lane;
  if (d > my_lane_center - 2 && d < my_lane_center + 2){
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    check_car_s = sensor_fusion[i][5];
    
    // if using previous points can project s value out
    check_car_s += ((double) prev_size * 0.02 * check_speed);  

    //check s values greater than mine and s gap
    if ((check_car_s > car_s) && (check_car_s-car_s) < 30){
      //do some logic here, lower reference velocity so we don't crash 
      // into the car in front of us, ould also flag to try to change
      // lanes
      // ref_vel = 29.5; //mph
      too_close = true;
      if (lane > 0) {
	lane = 0;
      }
    }
  } // end if car is in my lane
} // end for sensor fusion

if (too_close){
  ref_speed -= .224;
 }
 else if(ref_speed < 49.5){
   ref_speed += .224;
 }


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
  ptsx.push_back(ref_x);
  ptsx.push_back(ref_x_prev);

  ptsy.push_back(ref_y);
  ptsy.push_back(ref_y_prev);
  
}

// in Frenet add evenly 30m spaced points ahead of the the starting reference
vector double> next_wp0 = getXY(car_s + 30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector double> next_wp1 = getXY(car_s + 60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector double> next_wp2 = getXY(car_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);


ptsy.push_back(next_wp0[1)];
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

for (int i = 0; i < ptsx.size(); i++){
  
  //shift point to car reference
  // i.e. shift point such that car (ref_x, ref_y) is now at origin
  double shift_x = ptsx[i]-ref_x;
  double shift_y = ptsy[i]-ref_y;

  // rotate point so that ref_yaw is 0 degrees
  ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
  ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

}

// create spline
tk::spline s;

// set x,y points to the spline
s.set_points(ptsx, ptsy);

// define the actual (x,y) points we will use for the planner
vector <double> next_x_vals;
vector <double> next_y_vals;


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
// previous points, here we will always output 50 points
for (int i=0; i<=50-prev_size; i++){
  double N = target_dist / (0.02 * ref_vel / 2.24);  // 1 mph / 2.24 -> m/s
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
