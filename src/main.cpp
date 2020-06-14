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
#pragma comment(lib, "Ws2_32.lib")
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
		uWS::OpCode opCode) {
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
						//double car_x = j[1]["x"];

						double car_x = j[1]["x"];
						double car_y = j[1]["y"];
						double car_s = j[1]["s"];
						double car_d = j[1]["d"];
						double car_yaw = j[1]["yaw"];
						double car_speed = j[1]["speed"];
						auto previous_path_x = j[1]["previous_path_x"];
						auto previous_path_y = j[1]["previous_path_y"];
						double end_path_s = j[1]["end_path_s"];
						double end_path_d = j[1]["end_path_d"];
						vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
						/*std::sort(sensor_fusion.begin(),
							sensor_fusion.end(),
							[](const std::vector<double>& a, const std::vector<double>& b)
							{
								return a[3] < b[3];
							});*/


						vector<vector<double>>lanes_temp;
						vector<double> temp;


						vector<vector<vector<double>>> front;
						vector<vector<vector<double>>> back;
						front.push_back(lanes_temp);
						front.push_back(lanes_temp);
						front.push_back(lanes_temp);
						back.push_back(lanes_temp);
						back.push_back(lanes_temp);
						back.push_back(lanes_temp);

						//lanes.push_back(temp);
						//lanes.push_back(temp);
						//lanes.push_back(temp);


						//Expand sensor_fusion to contain lane number
						for (int i = 0; i < sensor_fusion.size(); i++)
						{
							float d = sensor_fusion[i][6];
							if (d < 4)
							{
								double distance = sensor_fusion[i][5] - car_s;
								sensor_fusion[i].push_back(distance);
								//lanes[0].push_back(i);
								if (distance > 0)
									front[0].push_back(sensor_fusion[i]);
								else
									back[0].push_back(sensor_fusion[i]);

							}
							else if (d < 8)
							{
								double distance = sensor_fusion[i][5] - car_s;
								sensor_fusion[i].push_back(distance);
								//lanes[1].push_back(i);
								if (distance > 0)
									front[1].push_back(sensor_fusion[i]);
								else
									back[1].push_back(sensor_fusion[i]);
							}
							else
							{
								double distance = sensor_fusion[i][5] - car_s;
								sensor_fusion[i].push_back(distance);
								//	lanes[2].push_back(i);
								if (distance > 0)
									front[2].push_back(sensor_fusion[i]);
								else
									back[2].push_back(sensor_fusion[i]);
							}
						}

						std::sort(front[0].begin(), front[0].end(), [](const std::vector<double>& a, const std::vector<double>& b) {	return a[7] < b[7];	});
						std::sort(front[1].begin(), front[1].end(), [](const std::vector<double>& a, const std::vector<double>& b) {	return a[7] < b[7];	});
						std::sort(front[2].begin(), front[2].end(), [](const std::vector<double>& a, const std::vector<double>& b) {	return a[7] < b[7];	});

						std::sort(back[0].begin(), back[0].end(), [](const std::vector<double>& a, const std::vector<double>& b) {	return a[7] > b[7];	});
						std::sort(back[1].begin(), back[1].end(), [](const std::vector<double>& a, const std::vector<double>& b) {	return a[7] > b[7];	});
						std::sort(back[2].begin(), back[2].end(), [](const std::vector<double>& a, const std::vector<double>& b) {	return a[7] > b[7];	});

						double ref_vel = 49.5; //mph
						double car_speed_mps = car_speed * 0.44704;

						int prev_size = previous_path_x.size();

						int next_wp = -1;
						double ref_x = car_x;
						double ref_y = car_y;
						double ref_yaw = deg2rad(car_yaw);

						if (prev_size < 2)
						{
							next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
						}
						else
						{
							ref_x = previous_path_x[prev_size - 1];
							double ref_x_prev = previous_path_x[prev_size - 2];
							ref_y = previous_path_y[prev_size - 1];
							double ref_y_prev = previous_path_y[prev_size - 2];
							ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
							next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

							car_s = end_path_s;

							car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / .02) * 2.237;
							car_speed_mps = car_speed * 0.44704;
						}

						//find ref_v to use
						bool overtake = false;
						double safe_distance = car_speed_mps;
						double overtake_speed = 49.5;

						vector<double> car_Front;
						//car is in front on my lane
						if (!front[lane].empty())
						{
							car_Front = front[lane][0];
							double vx = car_Front[3];
							double vy = car_Front[4];
							double check_speed = sqrt(vx * vx + vy * vy);

							car_Front.push_back(check_speed);

							double check_car_s = car_Front[5];
							check_car_s += ((double)prev_size * .02 * check_speed);

							double dist_s = check_car_s - car_s;

							car_Front.push_back(dist_s);
							car_Front.push_back(check_speed);

							//check s values greater than mine and s gap
							if (((dist_s) < 2 * safe_distance))
							{
								overtake = true;
								overtake_speed = check_speed * 2.237;
								if ((check_car_s - car_s) > safe_distance / 2)
								{
									//match that cars speed
									ref_vel = check_speed * 2.237;
								}
								else
								{
									//go slightly slower than the cars speed
									ref_vel = check_speed * 2.237 - 5;
								}
							}
						}

						bool overtake_done = false;

						//try to change lanes if too close to car in front
						if (overtake)
						{
							// try to change to left lane 
							if (lane != 0)
							{
								vector<double> car_F;
								vector<double> car_B;
								bool lane_safe_front = true;
								bool lane_safe_back = true;
								bool space = true;
								double safe_distance_Front = 5;
								double safe_distance_back = 5;


								if (!front[lane - 1].empty() && !back[lane - 1].empty())
								{
									car_F = front[lane - 1][0];
									car_B = back[lane - 1][0];
									if (car_F[5] - car_B[5] < 15)
										space = false;
								}

								//check car Front to the left
								if (!front[lane - 1].empty())
								{
									if (car_F.empty())
										car_F = front[lane - 1][0];

									//car is in left lane
									double vx = car_F[3];
									double vy = car_F[4];
									double check_speed = sqrt(vx * vx + vy * vy);


									double check_car_s = car_F[5];
									check_car_s += ((double)prev_size * .02 * check_speed);
									double dist_s = check_car_s - car_s;

									car_Front.push_back(dist_s);
									car_Front.push_back(check_speed);


									if ((car_speed_mps - check_speed) > safe_distance_Front)
										safe_distance_Front = car_speed_mps - check_speed;

									if (dist_s < safe_distance_Front || (check_speed < car_Front.back() + 5 && dist_s < (car_Front[7] + 5 + car_speed_mps - check_speed)))
									{
										lane_safe_front = false;
										std::cout << "Front Left false: " << (dist_s < safe_distance_Front) << " || " << (check_speed < car_Front.back()) << " && " << (dist_s < (car_Front[7] + 5 + car_speed_mps - check_speed)) << std::endl;
									}
								}

								//check car back to the left
								if (!back[lane - 1].empty())
								{
									if (car_B.empty())
										car_B = back[lane - 1][0];

									//car is in left lane
									double vx = car_B[3];
									double vy = car_B[4];
									double check_speed = sqrt(vx * vx + vy * vy);

									double check_car_s = car_B[5];
									check_car_s += ((double)prev_size * .02 * check_speed);
									double dist_s = check_car_s - car_s;
									car_Front.push_back(dist_s);
									car_Front.push_back(check_speed);
									if ((check_speed - car_speed_mps) > safe_distance_back)
										safe_distance_back = (check_speed - car_speed_mps);


									if (dist_s > -safe_distance_back)
									{
										lane_safe_back = false;
										std::cout << "Back Left false: " << std::endl;
										if (std::abs(dist_s) < std::abs(car_Front[7] - car_s) - 5)
										{
											ref_vel += 5;
											std::cout << "Accelerate. " << std::endl;
										}
									}
								}

								if ((lane_safe_front || lane_safe_back) && inCenter(lane, car_d))
								{
									if (!car_F.empty())
									{
										if (car_F[7] < safe_distance_Front)
										{
											if (car_Front[7] - car_F[7] > 15 && ref_vel < 49.5)
											{
												ref_vel += 49.5;
												std::cout << "Accelerate. " << std::endl;
											}
										}
										else if (car_B.empty())
										{
											ref_vel -= 5;
											std::cout << "Break. " << std::endl;
										}
										else if (space && (car_F[7] + car_B[7] < 0) && car_Front[7] > 10)
										{
											ref_vel += 5;
											std::cout << "Accelerate. " << std::endl;
										}
										else
										{
											ref_vel -= 5;
											std::cout << "Break. " << std::endl;
										}
									}
									else if(!car_B.empty())
									{
										if (car_Front[7] - car_B[7] > 15 && ref_vel < 49.5)
										{
											ref_vel += 49.5;
											std::cout << "Accelerate. " << std::endl;
										}
									}
									else
									{
										ref_vel -= 5;
										std::cout << "Break. " << std::endl;
									}
									
									overtake_done = true;
									lane -= 1;
									ref_vel = 49.5;
								}
							}
							// no left lane change posible try right
							if (lane != 2 && !overtake_done)
							{
								vector<double> car_F;
								vector<double> car_B;
								bool lane_safe_front = true;
								bool lane_safe_back = true;
								bool space = true;
								double safe_distance_Front = 5;
								double safe_distance_back = 5;


								if (!front[lane + 1].empty() && !back[lane - 1].empty())
								{
									car_F = front[lane + 1][0];
									car_B = back[lane + 1][0];
									if (car_F[5] - car_B[5] < 15)
										space = false;
								}

								//check car Front to the Right
								if (!front[lane + 1].empty())
								{
									if (car_F.empty())
										car_F = front[lane + 1][0];

									//car is in left lane
									double vx = car_F[3];
									double vy = car_F[4];
									double check_speed = sqrt(vx * vx + vy * vy);


									double check_car_s = car_F[5];
									check_car_s += ((double)prev_size * .02 * check_speed);
									double dist_s = check_car_s - car_s;

									car_Front.push_back(dist_s);
									car_Front.push_back(check_speed);


									if ((car_speed_mps - check_speed) > safe_distance_Front)
										safe_distance_Front = car_speed_mps - check_speed;

									if (dist_s < safe_distance_Front || (check_speed < car_Front.back() + 5 && dist_s < (car_Front[7] + 5 + car_speed_mps - check_speed)))
									{
										lane_safe_front = false;
										std::cout << "Front Left false: " << (dist_s < safe_distance_Front) << " || " << (check_speed < car_Front.back()) << " && " << (dist_s < (car_Front[7] + 5 + car_speed_mps - check_speed)) << std::endl;
									}
								}

								//check car back to the Right
								if (!back[lane + 1].empty())
								{
									if (car_B.empty())
										car_B = back[lane + 1][0];

									//car is in left lane
									double vx = car_B[3];
									double vy = car_B[4];
									double check_speed = sqrt(vx * vx + vy * vy);

									double check_car_s = car_B[5];
									check_car_s += ((double)prev_size * .02 * check_speed);
									double dist_s = check_car_s - car_s;
									car_Front.push_back(dist_s);
									car_Front.push_back(check_speed);
									if ((check_speed - car_speed_mps) > safe_distance_back)
										safe_distance_back = (check_speed - car_speed_mps);


									if (dist_s > -safe_distance_back)
									{
										lane_safe_back = false;
										std::cout << "Back Left false: " << std::endl;
										if (std::abs(dist_s) < std::abs(car_Front[7] - car_s) - 5)
										{
											ref_vel += 5;
											std::cout << "Accelerate. " << std::endl;
										}
									}
								}

								if ((lane_safe_front || lane_safe_back) && inCenter(lane, car_d))
								{
									if (!car_F.empty())
									{
										if (car_F[7] < safe_distance_Front)
										{
											if (car_Front[7] - car_F[7] > 15 && ref_vel < 49.5)
											{
												ref_vel += 49.5;
												std::cout << "Accelerate. " << std::endl;
											}
										}
										else if (car_B.empty())
										{
											ref_vel -= 5;
											std::cout << "Break. " << std::endl;
										}
										else if (space && (car_F[7] + car_B[7] < 0) && car_Front[7] > 10)
										{
											ref_vel += 5;
											std::cout << "Accelerate. " << std::endl;
										}
										else
										{
											ref_vel -= 5;
											std::cout << "Break. " << std::endl;
										}
									}
									else if (!car_B.empty())
									{				
										if (car_Front[7] - car_B[7] > 15 && ref_vel < 49.5)
										{
											ref_vel += 49.5;
											std::cout << "Accelerate. " << std::endl;
										}
									}
									else
									{
										ref_vel -= 5;
										std::cout << "Break. " << std::endl;
									}
									
									overtake_done = true;
									lane -= 1;
									ref_vel = 49.5;
								}
								/*bool lane_safe = true;


								//check car Front to the right
								if (!front[lane + 1].empty())
								{
									vector<double> car_F = front[lane + 1][0];

									//car is in left lane
									double vx = car_F[3];
									double vy = car_F[4];
									double check_speed = sqrt(vx * vx + vy * vy);

									car_F.push_back(check_speed);

									double check_car_s = car_F[5];
									check_car_s += ((double)prev_size * .02 * check_speed);
									double dist_s = check_car_s - car_s;

									//if ((dist_s < 2 * safe_distance && check_speed < car_Front.back()) || dist_s < car_Front[7] + 2 * car_speed_mps)
									double safe_distance_Front = 5;
									if ((car_speed_mps - check_speed) > safe_distance_Front)
										safe_distance_Front = car_speed_mps - check_speed;
									if (dist_s < safe_distance_Front || (check_speed < car_Front.back() + 5 && dist_s < (car_Front[7] + 5 + car_speed_mps - check_speed)))
									{
										lane_safe = false;
										std::cout << "Front Right false: " << (dist_s < safe_distance_Front) << " || " << (check_speed < car_Front.back()) << " && " << (dist_s < (car_Front[7] + 5 + car_speed_mps - check_speed)) << std::endl;
										if (dist_s < safe_distance_Front)
										{
											if (car_Front[7] - dist_s > 10 && ref_vel < 49.5)
											{
												ref_vel += 49.5;
												std::cout << "Accelerate. " << std::endl;
											}
											else
											{
												ref_vel -= 5;
												std::cout << "Brake. " << std::endl;
											}
										}
									}
								}

								//check car back to the right
								if (!back[lane + 1].empty())
								{
									vector<double> car_B = back[lane + 1][0];

									//car is in left lane
									double vx = car_B[3];
									double vy = car_B[4];
									double check_speed = sqrt(vx * vx + vy * vy);

									car_B.push_back(check_speed);

									double check_car_s = car_B[5];
									check_car_s += ((double)prev_size * .02 * check_speed);
									double dist_s = check_car_s - car_s;

									double safe_distance_back = 5;
									if ((check_speed - car_speed_mps) > safe_distance_back)
										safe_distance_back = (check_speed - car_speed_mps);


									if (dist_s > -safe_distance_back)
									{
										lane_safe = false;
										std::cout << "Back Right false: " << std::endl;
										if (std::abs(dist_s) < std::abs(car_Front[7] - car_s) - 5)
										{
											ref_vel += 5;
											std::cout << "Accelerate. " << std::endl;
										}
									}
								}

								if (lane_safe && inCenter(lane, car_d))
								{
									lane += 1;
									ref_vel = 49.5;
								}//*/

							}
						}

						if (overtake)
							goto Overtaking;

						if (lane != 2 && !overtake && (car_speed > 40))
						{
							bool lane_safe = true;
							if (!front[lane + 1].empty())
							{
								double vx = front[lane + 1][0][3];
								double vy = front[lane + 1][0][4];
								double check_speed = sqrt(vx * vx + vy * vy);

								double check_car_s = front[lane + 1][0][5];
								check_car_s += ((double)prev_size * .02 * check_speed);
								double dist_s = check_car_s - car_s;

								if (dist_s < safe_distance * 2.2)
								{
									lane_safe = false;
								}
							}
							if (!back[lane + 1].empty())
							{
								double vx = back[lane + 1][0][3];
								double vy = back[lane + 1][0][4];
								double check_speed = sqrt(vx * vx + vy * vy);

								double check_car_s = back[lane + 1][0][5];
								check_car_s += ((double)prev_size * .02 * check_speed);
								double dist_s = check_car_s - car_s;

								double safe_distance_back = 5;
								if ((check_speed - car_speed_mps) > safe_distance_back)
									safe_distance_back = (check_speed - car_speed_mps);

								if (dist_s > -safe_distance_back)
								{
									lane_safe = false;
								}
							}
							if (lane_safe && inCenter(lane, car_d))
							{
								lane += 1;
								ref_vel = 49.5;
							}

						}//*/

					Overtaking:
						vector<double> ptsx;
						vector<double> ptsy;

						if (prev_size < 2)
						{
							double prev_car_x = car_x - cos(car_yaw);
							double prev_car_y = car_y - sin(car_yaw);

							ptsx.push_back(prev_car_x);
							ptsx.push_back(car_x);

							ptsy.push_back(prev_car_y);
							ptsy.push_back(car_y);
						}
						else
						{
							ptsx.push_back(previous_path_x[prev_size - 2]);
							ptsx.push_back(previous_path_x[prev_size - 1]);

							ptsy.push_back(previous_path_y[prev_size - 2]);
							ptsy.push_back(previous_path_y[prev_size - 1]);


						}

						vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

						ptsx.push_back(next_wp0[0]);
						ptsx.push_back(next_wp1[0]);
						ptsx.push_back(next_wp2[0]);

						ptsy.push_back(next_wp0[1]);
						ptsy.push_back(next_wp1[1]);
						ptsy.push_back(next_wp2[1]);


						for (int i = 0; i < ptsx.size(); i++)
						{

							//shift car reference angle to 0 degrees
							double shift_x = ptsx[i] - ref_x;
							double shift_y = ptsy[i] - ref_y;

							ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
							ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

						}


						tk::spline s;


						s.set_points(ptsx, ptsy);

						vector<double> next_x_vals;
						vector<double> next_y_vals;

						for (int i = 0; i < previous_path_x.size(); i++)
						{
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

						double target_x = 30.0;
						double target_y = s(target_x);
						double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

						double x_add_on = 0;

						for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

							if (ref_vel > car_speed)
							{
								car_speed += .224;
							}
							else if (ref_vel < car_speed)
							{
								car_speed -= .224;
							}


							double N = (target_dist / (.02 * car_speed / 2.24));
							double x_point = x_add_on + (target_x) / N;
							double y_point = s(x_point);

							x_add_on = x_point;

							double x_ref = x_point;
							double y_ref = y_point;

							x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
							y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

							x_point += ref_x;
							y_point += ref_y;


							next_x_vals.push_back(x_point);
							next_y_vals.push_back(y_point);
						}

						json msgJson;
						msgJson["next_x"] = next_x_vals;
						msgJson["next_y"] = next_y_vals;

						auto msg = "42[\"control\"," + msgJson.dump() + "]";

						//this_thread::sleep_for(chrono::milliseconds(1000));
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

					}
				}
				else {
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
		char* message, size_t length) {
			ws.close();
			std::cout << "Disconnected" << std::endl;
		});

	int port = 4567;
	if (h.listen("127.0.0.1", port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}