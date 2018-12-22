#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

double polyder(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
        result += i*coeffs[i] * pow(x, i-1);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    
    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }
    
    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

std::tuple<double, double> toLocalCoordinates(double x_p, double y_p, double px, double py, double psi){
    double valx = (x_p - px) * cos(psi) + (y_p - py) * sin(psi);
    double valy = (y_p - py) * cos(psi) - (x_p - px) * sin(psi);
    
    return std::make_tuple(valx, valy);
}

std::tuple<vector<double>, vector<double>> toLocalCoordinates(vector<double> xs, vector<double> ys,
                                                              double px, double py, double psi){
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    assert(xs.size() == ys.size());
    
    for (int i = 0; i < xs.size(); i++) {
        auto tuple = toLocalCoordinates(xs.at(i), ys.at(i), px, py, psi);
        
        next_x_vals.push_back(std::get<0>(tuple));
        next_y_vals.push_back(std::get<1>(tuple));
    }
    
    return std::make_tuple(next_x_vals, next_y_vals);
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  Config config;
  
  config.ref_v = 12;
  config.ref_cte = 0.;
  config.ref_epsi = 0.;
  config.solver_N = 15;
  config.solver_dt = 0.1;

  config.w_cte = 2;
  config.w_epsi = 4;
  config.w_v = 4;
  
  config.w_delta = 200;
  config.w_a = 92;
  
  config.w_deltadot = 35;
  config.w_adot = 45;
  
  config.delay = 0.1;
  
  MPC mpc(config);
  
  h.onMessage([&mpc, &config](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.44704; //to m/s
          
          double steer_angle = double(j[1]["steering_angle"])*deg2rad(25);
          double throttle = double(j[1]["throttle"]);
          
          cout << "px " <<  px << " py " << py << " psi " << psi << " v " << v << endl;
          
          /*
           * TODO: Calculate steeering angle and throttle using MPC.
           *
           * Both are in between [-1, 1].
           *
           */
          auto nextPoints = toLocalCoordinates(ptsx, ptsy, px, py, psi);
          vector<double> next_x = std::get<0>(nextPoints);
          vector<double> next_y = std::get<1>(nextPoints);
          
          Eigen::VectorXd xs = Eigen::VectorXd::Map(&next_x[0], ptsx.size());
          Eigen::VectorXd ys = Eigen::VectorXd::Map(&next_y[0], ptsy.size());
          
          auto coeffs = polyfit(xs, ys, 3);
          
          double cte = polyeval(coeffs, 0) - 0;
          double epsi = 0 - atan(polyder(coeffs, 0));
          
          Eigen::VectorXd state(8);
          state << 0, 0, 0, v, cte, epsi, steer_angle, throttle;
          
          vector<double> vars = mpc.Solve(state, coeffs);
          
          
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          
          double steer_value = -vars[6] / deg2rad(25);
          double throttle_value = vars[7];
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          //Display the MPC predicted trajectory
          int var_count = 8;
          int points = (vars.size() - var_count)/2;
          int end_x_idx = var_count+points;
          
          vector<double> pts_mpc_x = vector<double>(vars.begin()+var_count, vars.begin()+end_x_idx);
          vector<double> pts_mpc_y = vector<double>(vars.begin()+end_x_idx, vars.begin()+end_x_idx+points);
            
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = pts_mpc_x;
          msgJson["mpc_y"] = pts_mpc_y;
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(int(config.delay*1000)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
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