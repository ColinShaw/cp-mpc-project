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


using json = nlohmann::json;


const int    waypoint_points   = 15;
const double waypoint_distance = 3.0;


string hasData(string s) 
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) 
  {
    return "";
  } 
  else if (b1!=string::npos && b2!=string::npos) 
  {
    return s.substr(b1, b2-b1+2);
  }
  return "";
}


double polyeval( Eigen::VectorXd coeffs, 
                 double x) {
  double result = 0.0;
  for (int i=0; i<coeffs.size(); i++) 
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


Eigen::VectorXd polyfit( Eigen::VectorXd xvals, 
                         Eigen::VectorXd yvals, 
                         int order ) {
  assert(xvals.size() == yvals.size());
  assert(order>=1 && order<=xvals.size()-1);
  Eigen::MatrixXd A(xvals.size(), order+1);

  for (int i=0; i<xvals.size(); i++) 
  {
    A(i, 0) = 1.0;
  }

  for (int j=0; j<xvals.size(); j++) 
  {
    for (int i=0; i<order; i++) 
    {
      A(j, i+1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main() 
{
  uWS::Hub h;

  MPC mpc;

  h.onMessage([&mpc]( uWS::WebSocket<uWS::SERVER> ws, 
                      char *data, 
                      int length,
                      uWS::OpCode opCode ) {

    string sdata = string(data).substr(0, length);

    if (sdata.size()>2 && sdata[0]=='4' && sdata[1]=='2') 
    {
      string s = hasData(sdata);
      if (s!="") 
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event=="telemetry") 
        {
          // x, y, psi, speed from telemetry
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];

          // Waypoint data as VectorXd (and size)
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          int pts_size = ptsx.size();

          // Convert to vehicle space
          Eigen::VectorXd ptsxd(pts_size);
          Eigen::VectorXd ptsyd(pts_size);
          for (int k=0; k<pts_size; k++)
          {
            double x = ptsx[k] - px;
            double y = ptsy[k] - py;

            ptsxd[k] = x*cos(psi) + y*sin(psi);
            ptsyd[k] = y*cos(psi) - x*sin(psi);
          }

          // Fit a polynomial 
          auto coeffs = polyfit(ptsxd, ptsyd, 3);

          // Generate cte and epsi estimate
          double cte  = polyeval(coeffs, 0.0);
          double epsi = atan(coeffs[1]);

          // Generate coordinate pairs to plot based on fit
          vector<double> next_x(waypoint_points);
          vector<double> next_y(waypoint_points);
          for (int k=0; k<waypoint_points; k++)
          {
            next_x[k] = waypoint_distance * (double)k;
            next_y[k] = polyeval(coeffs, next_x[k]);
          }

          // Create a state vector
          Eigen::VectorXd state(pts_size);
          state << 0.0, 0.0, 0.0, v, cte, epsi;	

          // Run MPC solver
          mpc.Solve(state, coeffs);

          // Telemetry communication
          json msgJson;

          // Actuators
          msgJson["steering_angle"] = mpc.steering_angle;
          msgJson["throttle"]       = mpc.throttle;

          // Waypoint trajectory
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;
          
          // MPC trajectory
          msgJson["mpc_x"] = mpc.mpc_x;
          msgJson["mpc_y"] = mpc.mpc_y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          //std::cout << msg << std::endl;

          this_thread::sleep_for(chrono::milliseconds(100)); 
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else 
      {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });


  h.onConnection([&h]( uWS::WebSocket<uWS::SERVER> ws, 
                       uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });


  h.onDisconnection([&h]( uWS::WebSocket<uWS::SERVER> ws, 
                          int  code,
                          char *message, 
                          int  length ) {
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
