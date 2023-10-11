// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "racing_control/racing_control.h"

#include <unistd.h>


RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) {
  if (!msg_process_) {
    msg_process_ = std::make_shared<std::thread>(
        std::bind(&RacingControlNode::MessageProcess, this));
  }
  this->declare_parameter<std::string>("pub_control_topic", pub_control_topic_);
  this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);

  this->declare_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);

  this->declare_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);

  this->declare_parameter<int>("bottom_threshold", bottom_threshold_);
  this->get_parameter<int>("bottom_threshold", bottom_threshold_);

  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);

  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);

  point_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PointStamped>(
      "racing_track_center_detection",
      10,
      std::bind(&RacingControlNode::subscription_callback_point,
      this,
      std::placeholders::_1)); 

  target_subscriber_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "racing_obstacle_detection",
      10,
      std::bind(&RacingControlNode::subscription_callback_target,
      this,
      std::placeholders::_1)); 

  publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, 5);

  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "RacingControlNode initialized!");
}

RacingControlNode::~RacingControlNode(){
  if (msg_process_ && msg_process_->joinable()) {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    while (!point_queue_.empty()) {
      point_queue_.pop();
    }
  }
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    while (!targets_queue_.empty()) {
      targets_queue_.pop();
    }
  }
}

void RacingControlNode::subscription_callback_point(const geometry_msgs::msg::PointStamped::SharedPtr point_msg){
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    point_queue_.push(point_msg);
    if (point_queue_.size() > 100) {
      point_queue_.pop();
      RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"),
                  "point_queue has cache point num > 100, drop the oldest "
                  "point message");
    }
  }
  return;
}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
  {
    sub_target_ = true;
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    targets_queue_.push(targets_msg);
    if (targets_queue_.size() > 100) {
      targets_queue_.pop();
      RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"),
                  "target_queue has cache target num > 100, drop the oldest "
                  "target message");
    }
  }
  return;
}

void RacingControlNode::MessageProcess(){
  while(process_stop_ == false){
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (!point_queue_.empty() && sub_target_ == false){
      auto point_msg = point_queue_.top();
      lock.unlock();
      LineFollowing(point_msg);
      lock.lock();
      point_queue_.pop();
    }
    if (!point_queue_.empty() && !targets_queue_.empty() && sub_target_== true) {
      auto point_msg = point_queue_.top();
      auto targets_msg = targets_queue_.top();
      if (point_msg->header.stamp == targets_msg->header.stamp) {
        lock.unlock();
        if(targets_msg->targets.size() == 0){
          LineFollowing(point_msg);
        } else {
            for(const auto &target : targets_msg->targets){
              int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
              if (bottom < bottom_threshold_){
                LineFollowing(point_msg);
              } else {
                ObstaclesAvoiding(target);
              }
            }
        }
        lock.lock();
        point_queue_.pop();
        targets_queue_.pop();
      } else if ((point_msg->header.stamp.sec > targets_msg->header.stamp.sec) ||
                  ((point_msg->header.stamp.sec == targets_msg->header.stamp.sec) &&
                  (point_msg->header.stamp.nanosec >
                    targets_msg->header.stamp.nanosec))) {
        targets_queue_.pop();
      } else {
        point_queue_.pop();
      }
    }
  }
}

void RacingControlNode::LineFollowing(const geometry_msgs::msg::PointStamped::SharedPtr point_msg){
  int x = int(point_msg->point.x);
  int y = int(point_msg->point.y);
  float temp = x - 320.0;
  if ((-20 < x ) && ( x < 0)) {
    temp = -20;
  } else if ((x > 0) && (x < 20)) {
    temp = 20;
  }
  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = follow_angular_ratio_ * temp / 150.0 * y / 224.0;
  twist_msg.linear.x = follow_linear_speed_;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
}

void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &target){
  auto twist_msg = geometry_msgs::msg::Twist();
  int center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float temp = center_x - 320.0;
  float angular_z = avoid_angular_ratio_ * 700 / temp;
  twist_msg.linear.x = avoid_linear_speed_;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
  return;
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
