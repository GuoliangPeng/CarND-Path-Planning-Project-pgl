//SF_YGH's submission. I did borrow some code from the video and github(Thanks to Christian's and James's repo)...
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <unordered_map>
#include <limits>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

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
  
  //start in lane 1;
  int lane = 1;

  //Have a reference velocity to target
  double ref_vel = 0.0; //mph  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &lane, &ref_vel]
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

    
//and now sensor fusion comes into play, the simulator is reporting list of all the other cars on the road. cars have s,d,x,y,vx,vy..

          int prev_size = previous_path_x.size(); //the simulator will tell what the previous path size is;

  

          if(prev_size>0)//trigger
          {
            //define car_s
            car_s=end_path_s;//previous path last point s
          }

          bool too_close = false;//initialize flag


          vector<Vehicle> ll_cars;      // Vector of left lane vehicles 
          vector<Vehicle> ml_cars;      // Vector of middle lane vehicles
          vector<Vehicle> rl_cars;      // Vector of right lane vehicles 
          // find ref_v to use
          bool switch_left = false; // initial flag for switching to left lane, true to trigger
          bool switch_right= false; // initial flag for switching to left lane, true to trigger

          for (int i=0; i<sensor_fusion.size(); i++)
          {
            // Add each car to appropriate vector list 
            
            float d = sensor_fusion[i][6]; //d
            //if(d<(2+4*lane+2)&& d> (2+4*lane-2))
                  //{
            double vx= sensor_fusion[i][3];
            double vy= sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s+=((double)prev_size*.02*check_speed);//if using previous points can project s value out
            // Push checked vehicle into convenient data structure for organizing later
            Vehicle check_car{check_car_s,d,check_speed}; 

            // Push vehicle to appropriate vector list 
            if (d > 0 && d <= 4) {ll_cars.push_back(check_car);}
            if (d > 4 && d <= 8) {ml_cars.push_back(check_car);}
            if (d > 8 && d <= 12) {rl_cars.push_back(check_car);}


            // Check if check_car is in same lane as your car 
            if(d < (2+4*lane+2) && d > (2+4*lane-2)) {
                    //check s values greater than mine and s gap
                    if((check_car_s>car_s)&&((check_car_s-car_s)<30))
                    {
						//do some logic here, lower reference velocity so we don't crash into the car in front of us, could also flag to try to change lanes.
                      //ref_vel=29.5;//mph
                      too_close = true;
                    }
                  }    
          }
		// Save vectors to hashtable for easy query
          unordered_map<int, vector<Vehicle>> lane_cars;
          lane_cars[0] = ll_cars;
          lane_cars[1] = ml_cars;
          lane_cars[2] = rl_cars;


          if (too_close) {
            ref_vel -= 0.224;           // incremental slowdown 

            // Check neighboring lanes 
            int right_lane = lane + 1;
            int left_lane = lane - 1;
            double dist1, dist2;

            // [1] Check if can switch to left lane
            if (left_lane >= 0) { // True if car is not in left most lane

              // find vehicle 1 and 2 in this lane
                /*
                |      |     |****
                |  v2  |     |****
                |      |  v  |****
                |      |     |****
                |  v1  |     |****
                */

              dist1 = numeric_limits<double>::max();
              dist2 = dist1;

              Vehicle vL1;
              Vehicle vL2;

              // Loop through all left lane vehcles 
              for (int i = 0; i < lane_cars[left_lane].size(); ++i) {
                
                // [a] Check if current car belongs to v1 category (i.e. behind our car)
                if (lane_cars[left_lane][i].s < car_s) {
                  // [a.1] from vehicles behild the car, find the vehicle that will be closest to the car.
                  if (abs(car_s - lane_cars[left_lane][i].s) < dist1) {
                    vL1 = lane_cars[left_lane][i];
                    dist1 = abs(car_s - vL1.s);
                  }
                }
                // [b] otherwise current car belongs to v2 category (i.e. in front of our car)
                else {
                  // [b.1] from vehicles in front of the car, find the vehicle that will be closest to the car.
                  if (abs(car_s - lane_cars[left_lane][i].s) < dist2) {
                    vL2 = lane_cars[left_lane][i];
                    dist2 = abs(car_s - vL2.s);
                  }
                
                }

              }

              // Determine if the gap between v2 and v1 is large enough to make the lange change
              if (((vL1.speed < car_speed && dist1 > 10) || dist1 > 30) &&
                  ((vL2.speed > car_speed && dist2 > 10) || dist2 > 30)) {
                switch_left = true;
              }
            }

            // [2] Check right lane in same way
            if (right_lane <= 2 && !switch_left) { // True if car is not in right most lane && 
                                                   // car will not be changing to left lane

              dist1 = numeric_limits<double>::max();
              dist2 = dist1;

              Vehicle vR1;
              Vehicle vR2;

              // Loop through all right lane vehcles 
              for (int i = 0; i < lane_cars[right_lane].size(); ++i) {
                
                // [a] Check if current car belongs to v1 category (i.e. behind our car)
                if (lane_cars[right_lane][i].s < car_s) {
                  // [a.1] from vehicles behild the car, find the vehicle that will be closest to the car.
                  if (abs(car_s - lane_cars[right_lane][i].s) < dist1) {
                    vR1 = lane_cars[right_lane][i];
                    dist1 = abs(car_s - vR1.s);
                  }
                }
                // [b] otherwise current car belongs to v2 category (i.e. in front of our car)
                else {
                  // [b.1] from vehicles in front of the car, find the vehicle that will be closest to the car.
                  if (abs(car_s - lane_cars[right_lane][i].s) < dist2) {
                    vR2 = lane_cars[right_lane][i];
                    dist2 = abs(car_s - vR2.s);
                  }
                
                }

              }

              // Determine if the gap between v2 and v1 is large enough to make the lange change
              if (((vR1.speed < car_speed && dist1 > 10) || dist1 > 30) &&
                  ((vR2.speed > car_speed && dist2 > 10) || dist2 > 30)) {
                switch_right = true;
              }
            }
            
            // [3] Update change lane flag is triggered
            if (switch_left) {lane--;}
            else if (switch_right) {lane++;}
          }
          else if (ref_vel < 49.5) { // cold start + incremental acceleration
            ref_vel += 0.224;
          }

          
        

          // Create a list of widely spaced (x,y) waypoints, evely spaced at 30m. (use sparsly spaced waypoints)
          //later we will interpolate these waypoints with a spline and fill it in with more points that control spline
          //create vector double list
          vector<double> ptsx;
          vector<double> ptsy;

          //reference x,y, yaw status
          //either we will reference the starting point as where the car is or at the previous paths and point

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //if previous size is almost empty, use the car state as starting reference
          if(prev_size <2)
          {
              //use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
              //notice here the ref_yaw still = car_yaw in rad.
          }
          //use the previous path's end point as starting reference
          else
          {
            //redefind reference state as previous path end point.
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            //change the ref_yaw by new calc.
            ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
            //use the two points make the path tangent to the previous path's end point.
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          //in Frenet add evenly 30m spaced points ahead of the starting reference,notice the map_waypoints_? is already loaded with the csv data in the main.cpp
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          //load the 3 points into ptsx,ptsy for spline calc
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //notice: before the spline take in place there is a function to transform the xy, to make the car heading to start from 0.
          for (int i = 0; i < ptsx.size(); i++)
          {
            //shift car ref angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          }          
          
          //and now create a spline 
          tk::spline s;
          //set (x,y) points to the spline
          s.set_points(ptsx,ptsy);
          //Define the actual (x,y) points we will use for the spline,already defined in main.cpp.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //store with all of the previous path points from last time
          for (int i = 0; i< previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //calc how to break up spline points so that we travel at our desired ref vel
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;//starts at 0
          // fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for (int i=1;i<=50 - previous_path_x.size();i++) {
          //now on visual, the takeaway is we can calc the N, and get the x(value), and tell the spline to get the y(s(x))

            double N = target_dist/(.02*ref_vel/2.24);//mph transformation using cont 2.24
            double x_point = x_add_on+(target_x)/N;//get the first x, so addon is used in here, a medium variable for iteration.
            double y_point = s(x_point); //get the first y

            x_add_on = x_point; //get the first x_add_on

            double x_ref = x_point; //def x_ref,bad naming? lol
            double y_ref = y_point; //def y_ref

            //rotate back to normal after rotating it earlier

            x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
//lanefollowing with spline ends
         
          json msgJson;
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
