#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "planner.h"
#include "data.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    Waypoints waypoints;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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
        waypoints.map_waypoints_x_.push_back(x);
        waypoints.map_waypoints_y_.push_back(y);
        waypoints.map_waypoints_s_.push_back(s);
        waypoints.map_waypoints_dx_.push_back(d_x);
        waypoints.map_waypoints_dy_.push_back(d_y);
    }

    // create planner
    Planner planner;

    h.onMessage([&waypoints, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    State state;

                    // Main car's localization Data
                    state.self_.car_x_ = j[1]["x"];
                    state.self_.car_y_ = j[1]["y"];
                    state.self_.car_s_ = j[1]["s"];
                    state.self_.car_d_ = j[1]["d"];
                    state.self_.car_yaw_ = j[1]["yaw"];
                    state.self_.car_speed_mph_ = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    for (auto it = previous_path_x.begin(); it != previous_path_x.end(); it++) {
                        state.previous_path_.path_x_.push_back(*it);
                    }
                    for (auto it = previous_path_y.begin(); it != previous_path_y.end(); it++) {
                        state.previous_path_.path_y_.push_back(*it);
                    }

                    // Previous path's end s and d values
                    state.previous_path_.end_path_s_ = j[1]["end_path_s"];
                    state.previous_path_.end_path_d_ = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); it++) {
                        auto other_car_values = *it;
                        Car other;
                        other.id_ = other_car_values[0];
                        other.car_x_ = other_car_values[1];
                        other.car_y_ = other_car_values[2];
                        other.car_s_ = other_car_values[3];
                        other.car_d_ = other_car_values[4];
                        other.car_yaw_ = other_car_values[5];
                        other.car_speed_mph_ = other_car_values[6];
                        state.others_.push_back(other);
                    }

                    json msgJson;

                    // call planner
                    Command command;
                    planner.plan(waypoints, state, command);

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = command.next_path_.path_x_;
                    msgJson["next_y"] = command.next_path_.path_y_;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
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
















































































