#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using namespace std;

//debug
//#define USERDEBUG

#ifdef USERDEBUG
#define Debug(x) cout << x
#else
#define Debug(x)
#endif

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steer, pid_throttle;
  // TODO: Initialize the pid variable.

  //debug info
  Debug( "[main]: Initialization begin: ====================" << endl);


  ///* Tuning reference
  /* tune the parameters by trial-and-error with refernce:
   * https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf
   * tuning process can be shown as below
   * Step 0: set Ki = Kd = 0, set a low Kp without oscillation
   * Step 1: TUNE Kd:
   *          Set Kd = 100*Kp as a starting point.
   *          If no oscillation, set Kd = Kd*2; If oscillation, set Kd = Kd/2;
   *          Once close to oscillation, fine tune Kd = Kd/2;
   * Step 2: TUNE Kp:
   *          Set Kp = Kd/100 as a starting point.
   *          If no oscillation, set Kp = Kp*10; If oscillation, set Kp = Kp/10;
   *          Once close to oscillation, fine tune Kp = Kp/2;
   * Step 3: TUNE Ki:
   *          Set Ki = (Kp/Kd) * Kp as a starting point.
   *          If no oscillation, set Ki= Ki*10; If oscillation, set Ki = Ki/10;
   *          Once close to oscillation, fine tune Ki = Ki/2;
   */

  /* My tuning for steer
   * Step 0.0: set Kp = 1, Ki = 0, Kd = 0. results: large oscillation --> unstable
   * Step 0.1: set Kp = 0.1, Ki = 0, Kd = 0. results: small oscillation --> unstable
   * Step 0.2: set Kp = 0.01, Ki = 0, Kd = 0. results: no oscillation ,but the system cannot respond quickly
   * Step 1.0: set Kp = 0.01, Ki = 0, Kd = 1. results: no oscillation, respond sluggishly, out of the lane
   * Step 1.1: set Kp = 0.01, Ki = 0, Kd = 2. results: no oscillation, respond sluggishly, out of the lane
   * Step 1.2: set Kp = 0.01, Ki = 0, Kd = 4. results: no oscillation, respond sluggishly, out of the lane
   * Step 1.3: set Kp = 0.01, Ki = 0, Kd = 10. results: within the lane, respond quick, change angle not often
   * Step 1.4: set Kp = 0.01, Ki = 0, Kd = 20. results: within the lane, respond quick, change angle frequently
   * Step 2.0: set Kp = 0.1, Ki = 0, Kd = 10. results: error decreased, no oscillation
   * Step 2.1: set Kp = 1, Ki = 0, Kd = 10. results: error decreased, sometimes oscillation
   * Step 2.2: set Kp = 0.5, Ki = 0, Kd = 10. results: sometimes oscillation
   * Step 2.2: set Kp = 0.2, Ki = 0, Kd = 10. results: no oscillation, within the lane
   * Step 3.0: set Kp = 0.2, Ki = 0.004, Kd = 10. results: lower static error, within the lane
   * Step 3.2: set Kp = 0.2, Ki = 0.04, Kd = 10. results: oscillation --> unstable
   * Step 3.2: set Kp = 0.2, Ki = 0.01, Kd = 10. results: a few oscillation, but still stable
   final setting: set Kp = 0.2, Ki = 0.004, Kd = 10
   Note: for this tuning, the most difficult part is to define the oscillation, since there is not tracking trajectory
   .to view. For further tuning, I shall plot the tracking trajectory for reference.
   */
  //Manual Tuned Parameters
  //double steer_Kp = 0.2;
  //double steer_Ki = 0.004;
  //double steer_Kd = 10.0;

  //Twiddle Tuned Parameters based on Manual Tuned Parameters
  double steer_Kp = 0.2;
  double steer_Ki = 0.001;
  double steer_Kd = 3.5;
  double steer_output = 1.0;
  Tunings steer_Tuning_type = Manual;
  pid_steer.Init(steer_Kp, steer_Ki, steer_Kd, steer_output, steer_Tuning_type);

  //for the speed tuning, it is more likely as thermal control.
  /* My tuning for throttle
   set Kp = 0.1, Ki = 0, Kd = 0 at first and find there is large static error
   set Kp = 0.1, Ki = 0, Kd = 1, system is unstable
   set Kp = 0.1, Ki = 0, Kd = 0.1, system becomes unstable
   set Kp = 0.1, Ki = 0.001, Kd = 0.1, large oscillation
   set Kp = 0.1, Ki = 0.0001, Kd = 0.1, no oscillation, but still static error
   set Kp = 0.1, Ki = 0.0002, Kd = 0.1, no oscillation, low static error
  //
  */
  //Manual Tuned Parameters
  //double throttle_Kp = 0.1;
  //double throttle_Ki = 0.0002;
  //double throttle_Kd = 0.1;

  //Twiddle Tuned Parameters based on Manual Tuned Parameters
  double throttle_Kp = 0.8;
  double throttle_Ki = 0.0002;
  double throttle_Kd = 0.11;
  double throttle_output = 1.0;
  Tunings throttle_Tuning_type = Manual;
  pid_throttle.Init(throttle_Kp, throttle_Ki, throttle_Kd, throttle_output, throttle_Tuning_type);

  //debug info
  Debug( "[main]: pid for steer are set as following: " << endl);
  Debug( "[main]: Kp_ = " << pid_steer.Kp_ << endl);
  Debug( "[main]: Ki_ = " << pid_steer.Ki_ << endl);
  Debug( "[main]: Kd_ = " << pid_steer.Kd_ << endl);

  Debug( "[main]: pid for throttle are set as following: " << endl);
  Debug( "[main]: Kp_ = " << pid_throttle.Kp_ << endl);
  Debug( "[main]: Ki_ = " << pid_throttle.Ki_ << endl);
  Debug( "[main]: Kd_ = " << pid_throttle.Kd_ << endl);

  int step = 0;
  h.onMessage([&step,&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          cout << "CTE: "<<cte <<endl;
          
          //control the steer based on cte error
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          if(steer_value > 1.0){
              steer_value = 1;
          }else if(steer_value < -1.0){
              steer_value = -1;
          }

          //control the throttle based on speed error
          // Speed is set to a constant value 25mph
          double speed_target = 20.0;
          if(step > 0){
            speed_target = 25;
          }
          
          if (cte > 0.8){
            speed_target = 25;
            step = 25;
            pid_throttle.Init(1, 0.05, 5, 1, Manual);
            
            
          }
          else if (cte > 1.5){
            speed_target = 15;
            step = 25;
            pid_throttle.Init(1.5, 0.05, 5, 1, Manual);

          }
          else if (cte > 2){
            speed_target = 1;
            step = 25;
            pid_throttle.Init(2, 0.05, 5, 1, Manual);
          }
          else{
            step--;
          }
          double speed_error = speed - speed_target;
          pid_throttle.UpdateError(speed_error);
          throttle_value = pid_throttle.TotalError();
          if(throttle_value > 1.0){
            throttle_value = 1.0;
          }else if(throttle_value < -1.0){
            throttle_value = -1.0;
          }
          cout << "steer_value: "<<steer_value<<endl;
          cout <<"throttle_value: "<<throttle_value<<endl;
          //debug
          Debug( "[main] ===================== : speed_error = " << speed_error << "      throttle_value = " << throttle_value << endl);

          // DEBUG
          Debug( "CTE: " << cte << " Steering Value: " << steer_value << endl);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          Debug( msg << endl);
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
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
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
