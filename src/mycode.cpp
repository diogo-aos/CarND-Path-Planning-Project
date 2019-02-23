	  
	  std::cout << "sensor_fusion" << std::endl;
	  for(int i=0; i<sensor_fusion.size(); i++){
	    std::cout << "id:" << sensor_fusion[i][0] << "\t s:" << sensor_fusion[i][5] << "\t d:" << sensor_fusion[i][6] << std::endl;
	  }
	  std::cout << "previous_path_x size " << previous_path_x.size() << std::endl;
	  std::cout << "previous_path_x " << previous_path_x << std::endl;




	  std::cout << "Car state:" << std::endl;
	  std::cout << "\tx: " << car_x << std::endl;
	  std::cout << "\ty: " << car_y << std::endl;
	  std::cout << "\ts: " << car_s << std::endl;
	  std::cout << "\td: " << car_d << std::endl;
	  std::cout << "\tyaw: " << car_yaw << std::endl;
	  std::cout << "\tspeed: " << car_speed << std::endl;
	  std::cout << "car_s:" << car_s << "\t car_d:" << car_d << std::endl;

	  // STATE MACHINE
	  enum State {KL, CR, CL};
	  State current = KL;
	  State next = KL;
	  switch(current){
	  case KL: {
	    break;
	  }
	  case CR: {
	    break;
	  }
	  case CL: {
	    break;
	  }
	  }  //end of switch
	  current = next;

	  std::cout << previous_path_x.size() << std::endl;

	  double time_inc = 0.02;
	  double T = 2;
	  int n_waypoints = 50;
	  double delta_s = TARGET_SPEED * T; // distance covered in T seconds at TARGET_SPEED
	  
	  vector<double> s_start; 
	  s_start.push_back(car_s);
	  s_start.push_back(0);
	  s_start.push_back(0); 

	  vector<double> s_end; 
	  s_end.push_back(car_s + delta_s);
	  s_end.push_back(TARGET_SPEED);
	  s_end.push_back(0); 

	  vector<double> s_coefs = JMT(s_start, s_end, T);
	  vector<double> next_s = poly5_vx(s_coefs, n_waypoints, time_inc);

	  double s_final = poly5_x(s_coefs, T);

	  std::cout << "delta_s: " << delta_s << std::endl;
	  std::cout << "s_final: " << s_final << std::endl;

	  std::cout << "s_start" << std::endl;
	  for(int i=0; i<s_start.size(); i++)
	    std::cout << "\tcoef " << i << ":" << s_start[i] << std::endl;

	  std::cout << "s_end" << std::endl;
	  for(int i=0; i<s_end.size(); i++)
	    std::cout << "\ts_end " << i << ":" << s_end[i] << std::endl;


	  std::cout << "s_coefs" << std::endl;
	  for(int i=0; i<s_coefs.size(); i++)
	    std::cout << "coef " << i << ":" << s_coefs[i] << std::endl;

	  std::cout << "next_s" << std::endl;
	  for(int i=0; i<next_s.size(); i++)
	    std::cout << "s " << i << ":" << next_s[i] << std::endl;

	  
	  vector<double> xy;
	  for(int i=0; i<next_s.size(); i++){
	    xy = getXY(next_s[i], car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_x_vals.push_back(xy[0]);
	    next_y_vals.push_back(xy[1]);
	  }

	  std::cout << "next_x_vals " << next_x_vals.size() << std::endl;
	  for(int i=0; i<next_x_vals.size(); i++)
	    std::cout << "\ti= " << i << ":" << next_x_vals[i] << std::endl;

