#ifndef CAR_H
#define CAR_H

#include "Variables.h"

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

  vector<double> predict(double dt){
    double p_s = s;
    double p_d;

    p_s += dt * v;
    
    vector <double> ret;
    ret.push_back(p_s);
    ret.push_back(p_d);
    return ret;
  }
  
};

#endif
