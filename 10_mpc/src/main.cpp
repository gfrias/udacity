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

Eigen::VectorXd PTSX(70);
Eigen::VectorXd PTSY(70);

std::tuple<Eigen::VectorXd, Eigen::VectorXd> getPath (double x, double y) {
  Eigen::VectorXd xs;
  Eigen::VectorXd ys;
  
  double min_d = MAXFLOAT;
  double min_i = -1;
  
  assert(PTSX.size() == PTSY.size());
  
  for (int i = 0; i < PTSX.size(); i++) {
    double px = PTSX[i];
    double py = PTSY[i];
    double dx = x-px;
    double dy = y-py;
    
    double d = dx*dx+dy*dy;
    
    if (d < min_d) {
      min_d = d;
      min_i = i;
    }
  }
  
  double size = PTSX.size() - min_i;
  if (6 < size)
    size = 6;
  
  
  assert(size > 0);
  xs = PTSX.segment(min_i, size);
  ys = PTSY.segment(min_i, size);
  
  return std::make_tuple(xs, ys);
}

double get_total_cte(Config config){
  MPC mpc(config);
  int iters = 50;
  
  // NOTE: free feel to play around with these
  double x = -40.6201;
  double y = 108.73;
  double psi = 3.73367;
  double v = 0;
  // The polynomial is fitted to a straight line so a polynomial with
  // order 1 is sufficient.
  auto tuple = getPath(x, y);
  Eigen::VectorXd ptsx = std::get<0>(tuple);
  Eigen::VectorXd ptsy = std::get<1>(tuple);
  
  auto coeffs = polyfit(ptsx, ptsy, 3);
  
  
  // The cross track error is calculated by evaluating at polynomial at x, f(x)
  // and subtracting y.
  double cte = polyeval(coeffs, x) - y;
  double epsi = psi - atan(polyder(coeffs, x));
  
  Eigen::VectorXd state(8);
  state << x, y, psi, v, cte, epsi, 0, 0;
  
  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};
  
  double total_cte = 0;
  for (size_t i = 0; i < iters; i++) {
//    std::cout << "Iteration " << i << std::endl;
    
    x = state(0);
    y = state(1);
    psi = state(2);
    v = state(3);
    double delta = state(6);
    double a = state(7);
    
    tuple = getPath(x, y);
    ptsx = std::get<0>(tuple);
    ptsy = std::get<1>(tuple);
    
    
//    cout << "for point(" << x << "," << y << ")" << endl;
//    cout << "path is ";
//    for (int i = 0; i < ptsx.size(); i++){
//      cout << "(" << ptsx(i) << "," << ptsy(i) << "), ";
//    }
//    cout << endl;
    
    coeffs = polyfit(ptsx, ptsy, 3);
    
    // The cross track error is calculated by evaluating at polynomial at x, f(x)
    // and subtracting y.
    cte = polyeval(coeffs, x) - y;
    epsi = psi - atan(polyder(coeffs, x));
    
    state << x, y, psi, v, cte, epsi, delta, a;
    
    auto vars = mpc.Solve(state, coeffs);
    
    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    total_cte += vars[4]*vars[4];
    
    epsi_vals.push_back(vars[5]);
    
    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);
    
    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5], vars[6], vars[7];
//    std::cout << "x = " << vars[0] << std::endl;
//    std::cout << "y = " << vars[1] << std::endl;
//    std::cout << "psi = " << vars[2] << std::endl;
//    std::cout << "v = " << vars[3] << std::endl;
//    std::cout << "cte = " << vars[4] << std::endl;
//    std::cout << "epsi = " << vars[5] << std::endl;
//    std::cout << "delta = " << vars[6] << std::endl;
//    std::cout << "a = " << vars[7] << std::endl;
//    std::cout << std::endl;
  }
  
  std::cout << "total cte " << total_cte << std::endl;
  return total_cte;
}

Config build_conf(Eigen::VectorXd p){
  Config config;
  config.ref_v = 10;
  config.ref_cte = 0.;
  config.ref_epsi = 0.;
  config.solver_N = 15;
  config.solver_dt = 0.1;
  
  config.w_cte = p[0];
  config.w_epsi = p[1];
  config.w_v = p[2];
  config.w_delta = p[3];
  config.w_a = p[4];
  config.w_deltadot = p[5];
  config.w_adot = p[6];
  
  config.delay = 0.1;
  
  cout << "p " << p << endl;

  return config;
}

double sum(Eigen::VectorXd v){
  double s = 0;
  
  for (int i = 0; i < v.size(); i++){
    s += v(i);
  }
  cout << "s " << s << endl;
  return s;
}

Eigen::VectorXd twiddle() {
  Eigen::VectorXd p(7);
//  p << 1,
//  1,
//  1,
//  1,
//  1,
//  1,
//  1;
  
  p << 2,
  2,
  4,
  92,
  92,
  45,
  45;
  
  Eigen::VectorXd dp(7);
  dp << 0.1, 0.1, 0.1, 1, 1, 1, 1;

  Config config = build_conf(p);
  double best_err = get_total_cte(config);
  double err;
  
  while (sum(dp) > 0.2) {
    cout << "test" << endl;
    for (int i = 0; i < 7; i++) {
      p[i] += dp[i];
      config = build_conf(p);
      err = get_total_cte(config);
      
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];
        config = build_conf(p);
        err = get_total_cte(config);
        
        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
          
        }
      }
    }
  }
  
  return p;
}

Eigen::VectorXd explore(){
  Eigen::VectorXd p(7);
  
    p << 1,
    1,
    1,
    1,
    1,
    1,
    1;
  
  Eigen::VectorXd best(7);
  
  best = p;
  
  Eigen::VectorXd dp(7);
  dp << 1, 1, 1, 1, 1, 1, 1;
  
  Eigen::VectorXd limits(7);
  limits << 10, 10, 10, 100, 100, 50, 50;
  
  Config config = build_conf(p);
  double best_err = get_total_cte(config);
  double err;
  
  int iters = 100;
  
  for (int i = 0; i < iters; i++){
    for (int j = 0; j < p.size(); j++){
      int lim = limits(j);
      int val = rand() % lim + 1;
      p[j] = double(val);
    }
    
    Config config = build_conf(p);
    err = get_total_cte(config);
    
    cout << i << " err " << err << " p " << p << endl << endl;
    if (err < best_err) {
      best_err = err;
      best = p;
    }
  }
  
  return best;
}

void find_params() {
  PTSX << 179.3083, 172.3083, 165.5735, 151.7483, 133.4783, 114.8083, 103.8935, 94.21827, 85.37355, 70.40827, 61.24355, 45.30827, 36.03354, 15.90826, 5.64827, -9.99173, -24.01645, -32.16173, -43.49173, -61.09, -78.29172, -93.05002, -107.7717, -123.3917, -134.97, -145.1165, -158.3417, -164.3164, -169.3365, -175.4917, -176.9617, -176.8864, -175.0817, -170.3617, -164.4217, -158.9417, -146.6317, -134.6765, -126.4965, -108.5617, -95.20645, -73.86172, -42.38173, 47.20827, 75.06355, 91.19354, 107.3083, 114.2635, 122.5583, 126.6183, 129.1083, 129.1283, 126.8983, 122.3383, 117.2083, 98.34827, 83.63827, 79.68355, 78.52827, 77.04827, 77.87827, 81.37827, 88.33827, 95.31827, 102.9283, 118.5435, 133.2435, 156.5083, 165.7435, 175.9083;
  
  PTSY << 98.67102, 117.181, 127.2894, 140.371, 150.771, 156.621, 158.4294, 158.891, 158.731, 157.101, 155.4194, 151.201, 148.141, 140.121, 135.321, 127.081, 119.021, 113.361, 105.941, 92.88499, 78.73102, 65.34102, 50.57938, 33.37102, 18.404, 4.339378, -17.42898, -30.18062, -42.84062, -66.52898, -76.85062, -90.64063, -100.3206, -115.129, -124.5206, -131.399, -141.329, -147.489, -150.849, -155.499, -157.609, -159.129, -159.099, -148.129, -143.929, -139.979, -134.209, -130.6506, -123.779, -118.159, -108.669, -100.349, -89.95898, -79.97897, -69.827, -42.02898, -20.72898, -12.66062, -7.878983, -1.338982, 5.75, 12.86102, 19.95102, 25.33102, 29.88102, 38.04939, 45.58102, 59.27102, 66.92102, 79.57102;
  
  srand (time(NULL));
//  Eigen::VectorXd ret = twiddle();
//  Eigen::VectorXd ret = explore();
//  std::cout << "best result " << ret << std::endl;
  
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
            cout << "delay " << int(config.delay*1000) << endl;
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
