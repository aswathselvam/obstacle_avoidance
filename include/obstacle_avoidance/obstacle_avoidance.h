/**
 * MIT License
 *
 * Copyright (c) 2021 Aswath Muthuselvam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file obstacle_avoidance.hpp
 * @author Aswath Muthuselvam
 * @brief A class to perform obstacle avoidance
 * @version 1.0
 * @date 11-29-20212
 * @copyright Copyright (c) 2021
 *
 */
#ifndef INCLUDE_OBSTACLE_AVOIDANCE_OBSTACLE_AVOIDANCE_H_
#define INCLUDE_OBSTACLE_AVOIDANCE_OBSTACLE_AVOIDANCE_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>

class Roam {
 public:
  /**
   * @brief explicit constructor for Talker object
   * @param nh pointer to nodehandle
   */
  explicit Roam(ros::NodeHandle* nh);

  /**
   * @brief Delete the NodeHandle
   */
  ~Roam();

  ros::NodeHandle* nh_;  // NodeHandle pointer

 private:
  int fov_degrees_;             // Field of view in degrees = 0 to 360 degrees
  float threshold_;             // Threshold of the distance
  ros::Publisher pub_cmd_vel_;  // Velocity publisher
  geometry_msgs::Twist vel_;    // Velocity twist message

  int rate_;  // Rate of program
  /**
   * @brief Function to init params
   */
  ros::Subscriber sub_scan_;  // Scan subscriber

  /**
   * @brief Function to Iniitalize variables and set ROS publisher and
   * subscriber
   */
  void Init();

  /**
   * @brief Main control loop of the program.
   */
  void Start();

  /**
   * @brief Perform operations on received scan data.
   * @param msg
   */
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief Function to detect obstacle from scan points and take action.
   * @param scan Laser scan
   * @return geometry_msgs::Twist message to
   */
  geometry_msgs::Twist React(const sensor_msgs::LaserScan& scan);
};
#endif  // INCLUDE_OBSTACLE_AVOIDANCE_OBSTACLE_AVOIDANCE_H_
