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

Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt, double lf) {
    Eigen::VectorXd next_state(state.size());
    auto x = state(0);
    auto y = state(1);
    auto psi = state(2);
    auto v = state(3);

    auto delta = actuators(0);
    auto a = actuators(1);

    // Recall the equations for the model:
    // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    // v_[t+1] = v[t] + a[t] * dt
    next_state(0) = x + v * cos(psi) * dt;
    next_state(1) = y + v * sin(psi) * dt;
    next_state(2) = psi + v / lf * delta * dt;
    next_state(3) = v + a * dt;
    return next_state;
}

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (!s.empty()) {
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

                    double delta = j[1]["steering_angle"];
                    double a = j[1]["throttle"];


                    /*
                    * TODO: Calculate steering angle and throttle using MPC.
                    *
                    * Both are in between [-1, 1].
                    *
                    */

                    const long n_waypts = ptsx.size();
                    const double dt = mpc.dt;
                    const double lf = mpc.Lf;

                    // Convert to the vehicle coordinate system
                    const double cos_psi = cos(-psi);
                    const double sin_psi = sin(-psi);
                    Eigen::VectorXd x_veh(n_waypts);
                    Eigen::VectorXd y_veh(n_waypts);
                    for (int i = 0; i < n_waypts; i++) {
                        const double dx = ptsx[i] - px;
                        const double dy = ptsy[i] - py;
                        x_veh[i] = dx * cos_psi - dy * sin_psi;
                        y_veh[i] = dy * cos_psi + dx * sin_psi;
                    }




//                    double cte = polyeval(coeffs, 0);
                    // x = 0, so
//                    const double epsi = -atan(coeffs[1]); //-f'(0)

                    // Kinematic model is used to predict vehicle state at the actual
                    // moment of control (current time + delay dt)
                    // Recall the equations for the model:
                    // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
                    // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
                    // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
                    // v_[t+1] = v[t] + a[t] * dt
                    //cos(psi[t]) = 1 | x=0 | y=0
//                    double pred_px = 0.0 + v * dt;
//                    const double pred_py = 0.0;
//                    double pred_psi = v * -delta / Lf * dt;
//                    double pred_v = v + a * dt;
//                    double pred_cte = cte + (v * sin(epsi) * dt);
//                    double pred_epsi = epsi + (v * -delta / Lf * dt);

                    auto coeffs = polyfit(x_veh, y_veh, 3); // Fit waypoints


                    // Initial state.
                    const double x0 = 0;
                    const double y0 = 0;
                    const double psi0 = 0;
                    const double cte0 = coeffs[0];
                    const double epsi0 = -atan(coeffs[1]);

                    // State after delay.
//                    double pred_px = x0 + (v * cos(psi0) * dt);
//                    double pred_py = y0 + (v * sin(psi0) * dt);
//                    double pred_psi = psi0 - (v * delta * dt / lf);
//                    double pred_v = v + a * dt;
//                    double pred_cte = cte0 + (v * sin(epsi0) * dt);
//                    double pred_epsi = epsi0 - (v * atan(coeffs[1]) * dt / lf);
                    const double pred_px = v * dt;
                    const double pred_py = 0;
                    const double pred_psi = - v * delta * dt / lf;
                    const double pred_v = v + a * dt;
                    const double pred_cte = cte0 + v * sin(epsi0) * dt;
                    const double pred_epsi = epsi0 + pred_psi;

                    Eigen::VectorXd state0(4);
                    state0 << 0, 0, 0, v;
                    Eigen::VectorXd actuators(2);
                    actuators << coeffs[0], -atan(coeffs[1]);
                    auto kin = globalKinematic(state0, actuators, dt, lf);

                    // Feed in the predicted state values
                    Eigen::VectorXd state(6);
//                    state << kin[0], kin[1], kin[2], kin[3], pred_cte, pred_epsi;
                    state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;
                    auto mpc_sol = mpc.Solve(state, coeffs);

                    double steer_value = mpc_sol[0] / deg2rad(25); // convert to [-1..1] range
                    double throttle_value = mpc_sol[1];
                    /////////////////////////////////////

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = -steer_value;
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
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
