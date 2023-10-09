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

#ifndef RACING_CONTROL_H_
#define RACING_CONTROL_H_

#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ai_msgs/msg/perception_targets.hpp"



struct compare_point {
  bool operator()(const geometry_msgs::msg::PointStamped::SharedPtr f1,
                  const geometry_msgs::msg::PointStamped::SharedPtr f2) {
    return ((f1->header.stamp.sec > f2->header.stamp.sec) ||
            ((f1->header.stamp.sec == f2->header.stamp.sec) &&
             (f1->header.stamp.nanosec > f2->header.stamp.nanosec)));
  }
};
struct compare_target {
  bool operator()(const ai_msgs::msg::PerceptionTargets::SharedPtr m1,
                  const ai_msgs::msg::PerceptionTargets::SharedPtr m2) {
    return ((m1->header.stamp.sec > m2->header.stamp.sec) ||
            ((m1->header.stamp.sec == m2->header.stamp.sec) &&
             (m1->header.stamp.nanosec > m2->header.stamp.nanosec)));
  }
};

class RacingControlNode : public rclcpp::Node{
public:
  RacingControlNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RacingControlNode() override;
private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;

    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void subscription_callback_point(const geometry_msgs::msg::PointStamped::SharedPtr point_msg);
    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void LineFollowing(const geometry_msgs::msg::PointStamped::SharedPtr point_msg);
    void ObstaclesAvoiding(const ai_msgs::msg::Target &target);
    void MessageProcess(void);
    std::string pub_control_topic_ = "cmd_vel";

    std::priority_queue<geometry_msgs::msg::PointStamped::SharedPtr,
                      std::vector<geometry_msgs::msg::PointStamped::SharedPtr>,
                      compare_point>
    point_queue_;
    std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>,
                      compare_target>
    targets_queue_;

    std::mutex point_target_mutex_;
    bool process_stop_ = false;
    std::shared_ptr<std::thread> msg_process_;
    float avoid_angular_ratio_ = 1.1;
    float avoid_linear_speed_ = 0.25;
    bool sub_target_ = false; 
    int bottom_threshold_ = 340;
};


#endif  // RACING_CONTROL_H_