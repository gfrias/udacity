#include <uWS/uWS.h>
#include <cstring>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

const double MAXERR = 1E20;

// for convenience
using namespace std;
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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double calculate_steer_value(PID &pidSteering, double cte)
{
  pidSteering.UpdateError(cte);
  double steer_value = -(pidSteering.Kp*pidSteering.p_error + pidSteering.Kd*pidSteering.d_error + pidSteering.Ki*pidSteering.i_error)/10;
  
  //steering value is [-1, 1]
  if (steer_value > 1)
    steer_value = 1;
  else if (steer_value < -1)
    steer_value = -1;
  
  return steer_value;
}

double calculate_throttle_value(double speed, double steer_value)
{
  const double MAXSPEED = 20.0;
  
  double throttle_theory = (MAXSPEED-float(speed))*0.5;
  double throttle = throttle_theory*((1-0.8*fabs(steer_value))/1);
  
  if (throttle > -1 && throttle < 0) { //reduce number of little brakes
    throttle = 0;
  }
  
  return throttle;
}

string build_message(double steer_value, double throttle)
{
  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle;
  
  return "42[\"steer\"," + msgJson.dump() + "]";
}

string run(PID &pidSteering, double cte, double speed, double angle){
  double steer_value = calculate_steer_value(pidSteering, cte);
  double throttle_value = calculate_throttle_value(speed, steer_value);
  
  return build_message(steer_value, throttle_value);
}

string twiddle(PID &pidSteering, double cte, double speed, double angle){
  static double p[3] = {0, 0, 0};
  static double dp[3] = {1, 1, 1};
  
  static bool increase = true;
  
  static double best_err = MAXERR;
  
  const static double MAX_ALLOWED_COUNT = 600; //MAX ITERATIONS
  
  static int i = 0;
  static int idx = 0;
  
  //***************************************
  
  if (i == 0)
  {
    pidSteering = PID();
    p[idx] += dp[idx];
    pidSteering.Init(p[0], p[1], p[2]);
    cout << "trying with: " << p[0] << "," << p[1] << "," << p[2] << endl;
  }
  i++;
  
  string msg = run(pidSteering, cte, speed, angle);
  
  double err = pidSteering.TotalError();
  if (err > best_err){
    err = MAXERR;
    i = MAX_ALLOWED_COUNT;
  }
  
  if (i >= MAX_ALLOWED_COUNT) {
    double sum = dp[0] + dp[1] + dp[2];
    if (sum <= 0.02 and idx == 0) {
      if (err < best_err){
        best_err = err;
      }
      cout << "best found " << p[0] << "," << p[1] << "," << p[2] << " sum " << sum << " best err " << best_err << endl;
      return "";
    }
    
    cout << "for " << p[0] << "," << p[1] << "," << p[2] << " sum " << sum << " error is " << err << endl;
    
    if (err < best_err){
      best_err = err;
      
      dp[idx] *= 1.1;
      idx = (idx+1)%3;
      if (!increase) {
        increase = true;
      }
    } else {
      if (increase) {
        p[idx] -= 3*dp[idx];
      } else {
        p[idx] += dp[idx];
        dp[idx] *= 0.9;
        idx = (idx+1)%3;
      }
      
      increase = !increase;
    }
    
    i = 0;
    msg = "42[\"reset\",{}]";
    
  }
  
  return msg;
}

int check_params(bool& do_twiddle, int argc, char* argv[]) {
  if (argc > 2) {
    cout << "Wrong parameters: check --help" << endl << endl;
    return 255;
  } else if (argc == 2) {
    if (strcmp(argv[1], "--help") == 0) {
      cout << "PID control for Udacity Car simulator" << endl;
      cout << "run without parameters to control the car" << endl;
      cout << "run with --train to find the best parameters" << endl << endl;
      return 1;
    } else if (strcmp(argv[1], "--train") == 0) {
      do_twiddle = true;
      return 0;
    } else {
      cout << "Parameter unknown" << endl << endl;
      return 255;
    }
  }

  return 0;
}

int main(int argc, char* argv[])
{
  cout << fixed;
  cout << setprecision(2);
  
  uWS::Hub h;
  
  static bool do_twiddle = true;  //change this to train the PID or run the car
  int ret = check_params(do_twiddle, argc, argv);
  if (ret != 0) return ret;
  
  // Initialize the pid variable.
  PID pidSteering;
  pidSteering.Init(1.65, 0.02, 5.27);
  
  if (do_twiddle) {
    cout << "Begin parameters search (twiddle)" << endl;
    pidSteering.Init(0, 0, 0);
  }
  
  h.onMessage([&pidSteering](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());
          
          string msg;
          if (do_twiddle){
            msg = twiddle(pidSteering, cte, speed, angle);
          } else {
            msg = run(pidSteering, cte, speed, angle);
          }
          
          if (msg == "") {
            ws.close();
          }
          
          cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });
  
  int port = 4567;
  if (h.listen(port))
  {
    cout << "Listening to port " << port << endl;
  }
  else
  {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
