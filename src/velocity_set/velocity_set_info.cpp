/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <waypoint_planner/velocity_set/velocity_set_info.h>
#include <apsrc_msgs/AvpCommand.h>

VelocitySetInfo::VelocitySetInfo()
{
  ros::NodeHandle private_nh_("~");
  ros::NodeHandle nh;

  double vel_change_limit_kph = 9.972;
  private_nh_.param<double>("remove_points_upto", remove_points_upto_, 2.3);
  private_nh_.param<double>("stop_distance_obstacle", stop_distance_obstacle_, 10.0);
  private_nh_.param<double>("stop_distance_stopline", stop_distance_stopline_, 5.0);
  private_nh_.param<double>("detection_range", stop_range_, 1.3);
  private_nh_.param<int>("points_threshold", points_threshold_, 10);
  private_nh_.param<double>("detection_height_top", detection_height_top_, 0.2);
  private_nh_.param<double>("detection_height_bottom", detection_height_bottom_, -1.7);
  private_nh_.param<double>("acceleration", acceleration_, 0.5);
  private_nh_.param<double>("deceleration_obstacle", deceleration_obstacle_, 0.8);
  private_nh_.param<double>("deceleration_stopline", deceleration_stopline_, 0.6);
  private_nh_.param<double>("velocity_change_limit", vel_change_limit_kph, 9.972);
  private_nh_.param<double>("deceleration_range", deceleration_range_, 0);
  private_nh_.param<double>("temporal_waypoints_size", temporal_waypoints_size_, 100.0);
  velocity_change_limit_ = vel_change_limit_kph / 3.6;  // kph -> mps

  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh, private_nh_);
  health_checker_ptr_->ENABLE();
}

void VelocitySetInfo::clearPoints()
{
  points_.clear();
}

void VelocitySetInfo::configCallback(const autoware_config_msgs::ConfigVelocitySetConstPtr &config)
{
  stop_distance_obstacle_ = config->stop_distance_obstacle;
  stop_distance_stopline_ = config->stop_distance_stopline;
  stop_range_ = config->detection_range;
  points_threshold_ = config->threshold_points;
  detection_height_top_ = config->detection_height_top;
  detection_height_bottom_ = config->detection_height_bottom;
  deceleration_obstacle_ = config->deceleration_obstacle;
  deceleration_stopline_ = config->deceleration_stopline;
  velocity_change_limit_ = config->velocity_change_limit / 3.6;  // kmph -> mps
  deceleration_range_ = config->deceleration_range;
  temporal_waypoints_size_ = config->temporal_waypoints_size;
}

void VelocitySetInfo::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  health_checker_ptr_->CHECK_RATE("topic_rate_points_no_ground_slow", 8, 5, 1,
                                  "topic points_no_ground subscribe rate slow.");
  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  points_.clear();
  for (const auto &v : sub_points)
  {
    if (v.x == 0 && v.y == 0)
      continue;

    if (v.z > detection_height_top_ || v.z < detection_height_bottom_)
      continue;

    // ignore points nearby the vehicle
    if (v.x * v.x + v.y * v.y < remove_points_upto_ * remove_points_upto_)
      continue;

    points_.push_back(v);
  }
}

void VelocitySetInfo::detectionCallback(const std_msgs::Int32 &msg)
{
    wpidx_detectionResultByOtherNodes_ = msg.data;
}

void VelocitySetInfo::controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_ = *msg;

  if (!set_pose_)
    set_pose_ = true;
}

void VelocitySetInfo::setLocalizerPose(geometry_msgs::TransformStamped *map_to_lidar_tf)
{
    geometry_msgs::Pose lidarPose;
    geometry_msgs::Point lidarPoint;

    lidarPoint.x = map_to_lidar_tf->transform.translation.x;
    lidarPoint.y = map_to_lidar_tf->transform.translation.y;
    lidarPoint.z = map_to_lidar_tf->transform.translation.z;

    lidarPose.position = lidarPoint;
    lidarPose.orientation = map_to_lidar_tf->transform.rotation;

    localizer_pose_ = lidarPose;
}

void VelocitySetInfo::avpCommandCallback(const apsrc_msgs::AvpCommandConstPtr& msg)
{
  if (msg->stop_distance >= 5){
    stop_distance_obstacle_ = msg->stop_distance;  
  }
}