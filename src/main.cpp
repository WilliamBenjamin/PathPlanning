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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;



int main() {
  uWS::Hub h;
  int lane = 1;
  double ref_vel = 0.0;
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

  

  h.onMessage([&ref_vel, &lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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


          int previous_size = previous_path_x.size();
     
          if (previous_size > 0) {
              car_s = end_path_s;
            }
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          bool car_just_ahead = false;
          bool car_to_left = false;
          bool car_to_right = false;

          // Prediction: Predict the positions of other cars
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            // Get data about others cars
            double other_car_vx = sensor_fusion[i][3];
            double other_car_vy = sensor_fusion[i][4];
            float other_car_d = sensor_fusion[i][6];
            double other_car_s = sensor_fusion[i][5];

            // calculate speed of other car
            double other_car_speed = sqrt(other_car_vx*other_car_vx + 
                    other_car_vy*other_car_vy);

            // get the lane position of other car based on d val
            int other_car_lane = -1;
            if (other_car_d > 0 && other_car_d < 4 ) { // other car in lane 0
              other_car_lane = 0;
             
            } else if ( other_car_d > 4 && other_car_d < 8 ) { // other car in lane 1
              other_car_lane = 1;
              
            } else if ( other_car_d > 8 && other_car_d < 12 ) { // other car is in lane 2
              other_car_lane = 2;
              
            }else
            {
              
              continue;
            }

            other_car_s += ((double)previous_size*0.02*other_car_speed);
            // deduce the lane for the other car.
            if ( other_car_lane == lane ) {
                  car_just_ahead |= other_car_s > car_s && other_car_s - car_s < 30;
                } else if ( other_car_lane - lane == -1 ) {
                 car_to_left |= car_s - 30 < other_car_s && car_s + 30 > other_car_s;
                } else if ( other_car_lane - lane == 1 ) {
                 car_to_right |= car_s - 30 < other_car_s && car_s + 30 > other_car_s;
                }

          }

          if (car_to_right) std::cout << "CAR ON THE RIGHT" << std::endl;
					if (car_to_left) std::cout << "CAR ON THE LEFT" << std::endl;
					if (car_just_ahead) std::cout << "CAR JUST AHEAD!" << std::endl;


          // Behavior: Chosing the state of the vehicle
          const double MAX_SPEED = 49.25;
          const double MAX_ACCELERATION = .224;
          double speed_difference = 0;

          // simple state machine
          if ( car_just_ahead ) { // Change lanes if car straight ahead 
              if ( !car_to_left && lane > 0 ) {
                lane--;
                std::cout << "CHANGING LANES" << std::endl; 
              } else if ( !car_to_right && lane != 2 ){
                lane++;
                std::cout << "CHANGING LANES" << std::endl;
              } else {
                speed_difference -= MAX_ACCELERATION;
              }
            } else { 
              if ( lane != 1 ) { 
                if ( ( lane == 0 && !car_to_right ) || ( lane == 2 && !car_to_left ) ) {
                  lane = 1; // get to middle lane
                  std::cout << "CHANGING LANES" << std::endl;
                }
              }
              if ( ref_vel < MAX_SPEED ) {
                speed_difference += MAX_ACCELERATION; // accelerate
              }
            }

          // Trajectory prediction
            vector<double> spline_points_x;
            vector<double> spline_points_y;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if ( previous_size < 2 ) {
                
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                spline_points_x.push_back(prev_car_x);
                spline_points_x.push_back(car_x);

                spline_points_y.push_back(prev_car_y);
                spline_points_y.push_back(car_y);
            } else {
                
                ref_x = previous_path_x[previous_size - 1];
                ref_y = previous_path_y[previous_size - 1];

                double ref_x_prev = previous_path_x[previous_size - 2];
                double ref_y_prev = previous_path_y[previous_size - 2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                spline_points_x.push_back(ref_x_prev);
                spline_points_x.push_back(ref_x);

                spline_points_y.push_back(ref_y_prev);
                spline_points_y.push_back(ref_y);
            }

            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_points_x.push_back(next_wp0[0]);
            spline_points_y.push_back(next_wp0[1]);


            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_points_x.push_back(next_wp1[0]);
             spline_points_y.push_back(next_wp1[1]);


            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_points_x.push_back(next_wp2[0]);
            spline_points_y.push_back(next_wp2[1]);


            for ( int i = 0; i < spline_points_x.size(); i++ ) {
              double shift_x = spline_points_x[i] - ref_x;
              double shift_y = spline_points_y[i] - ref_y;

              spline_points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              spline_points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
            // fit a cubic spline using open source library from https://kluge.in-chemnitz.de/opensource/spline/
            tk::spline spline_trajectory;
            spline_trajectory.set_points(spline_points_x, spline_points_y);

            
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // Insert previous points 
            for ( int i = 0; i < previous_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // compute target 30m away
            double x_step = 0;
            double target_x = 30.0;
            double target_y = spline_trajectory(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            for( int i = 1; i < 50 - previous_size; i++ ) {
              ref_vel += speed_difference;
              if ( ref_vel > MAX_SPEED ) {
                ref_vel = MAX_SPEED;
              } else if ( ref_vel < MAX_ACCELERATION ) {
                ref_vel = MAX_ACCELERATION;
              }
              double number_of_points = target_dist/(0.02*ref_vel/2.24);
              double delta_x = x_step + target_x/number_of_points;
              double delta_y = spline_trajectory(delta_x);

              x_step = delta_x;

              double x_ref = delta_x;
              double y_ref = delta_y;

              delta_x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              delta_y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              double next_x = ref_x + delta_x;
              double next_y = ref_y + delta_y;

              next_x_vals.push_back(next_x);
              next_y_vals.push_back(next_y);
            }

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