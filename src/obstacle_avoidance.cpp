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
 * @file obstacle_avoidance.cpp
 * @author Aswath Muthuselvam
 * @brief A class to perform obstacle avoidance
 * @version 1.0
 * @date 11-29-20212
 * @copyright Copyright (c) 2021
 *
 */
#include <obstacle_avoidance/obstacle_avoidance.h>

Roam::Roam(ros::NodeHandle* nh) {
  this->nh_ = nh;
  Init();
  Start();
}

Roam::~Roam() { delete this->nh_; }

void Roam::Init() {
  this->pub_cmd_vel_ =
      this->nh_->advertise<geometry_msgs::Twist>("cmd_vel", this->rate_, this);
  this->sub_scan_ = this->nh_->subscribe("scan", 1, &Roam::ScanCallback, this);
  this->vel_.linear.x = 1;
  this->vel_.linear.y = 0;
  this->vel_.linear.z = 0;
  this->vel_.angular.x = 0;
  this->vel_.angular.y = 0;
  this->vel_.angular.z = 0;
  this->fov_degrees_ = 180;
  this->threshold_ = 1;
}

void Roam::Start() {
  ros::Rate loop_rate(this->rate_);
  while (ros::ok()) {
    ROS_DEBUG_STREAM("Roaming now");
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Roam::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO_STREAM("Received scan");
  this->vel_ = this->React(*msg);
  pub_cmd_vel_.publish(this->vel_);
}

geometry_msgs::Twist Roam::React(const sensor_msgs::LaserScan& scan) {
  bool make_turn = false;
  for (int i = 0; i < this->fov_degrees_ / 2; i++) {
    if ((scan.ranges[i] <= this->threshold_) ||
        (scan.ranges[359 - i] <= threshold_)) {
      make_turn = true;
      ROS_WARN_STREAM("Detected obstacle");
      break;
    }
  }
  if (make_turn) {
    this->vel_.linear.x = 0;
    this->vel_.angular.z = 0.5;
  } else {
    this->vel_.linear.x = 0.2;
    this->vel_.angular.z = 0;
  }
  return vel_;
}
