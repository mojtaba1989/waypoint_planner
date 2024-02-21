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

#ifndef WAYPOINT_PLANNER_VELOCITY_SET_VELOCITY_SET_PATH_H
#define WAYPOINT_PLANNER_VELOCITY_SET_VELOCITY_SET_PATH_H

#include <autoware_msgs/Lane.h>
#include <apsrc_msgs/EsrValid.h>
#include <libwaypoint_follower/libwaypoint_follower.h>

class VelocitySetPath
{
private:
  autoware_msgs::Lane original_waypoints_;
  autoware_msgs::Lane updated_waypoints_;
  autoware_msgs::Lane temporal_waypoints_;
  bool set_path_{false};
  double current_vel_{0.0};
  double target_vel_{0.0};
  uint8_t target_vel_conf_{0};
  geometry_msgs::Pose closest_path_pose_;
  bool use_fcr_{false};                         // enable radar based control
  double lead_speed_{0.0};                      // lead vehicle speed estimation
  double desired_time_gap_{2.0};                // desired time gap based on ego speed
  

  // ROS param
  double min_accel_velocity_;  // m/s
  double min_decel_velocity_;  // m/s
  float accel_velocity_;       // m/s

  bool checkWaypoint(int wp_num) const;

public:
  VelocitySetPath();
  ~VelocitySetPath() = default;

  void updateClosestPathPose(geometry_msgs::Pose current_pose, int closest_wp_index);
  double calcChangedVelocity(const double& current_vel, const double& accel, const std::array<int, 2>& range) const;
  void changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint, double deceleration, int stop_distance);
  void avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint);
  void avoidSuddenAcceleration(double acceleration, int closest_wp_index);
  void changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint);
  void setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, geometry_msgs::PoseStamped control_pose);
  void initializeNewWaypoints();
  void resetFlag();
  double calcStopPointVal(double deceleration, int stop_distance);

  // ROS Callbacks
  void waypointsCallback(const autoware_msgs::LaneConstPtr& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  void radarSpeedReadCallback(const  apsrc_msgs::EsrValid::ConstPtr& msg);

  double distanceBetweenWaypoints(const int& begin, const int& end) const;
  double distanceBetweenPoints(const geometry_msgs::Point& begin, const geometry_msgs::Point& end) const;

  autoware_msgs::Lane getPrevWaypoints() const
  {
    return original_waypoints_;
  }

  autoware_msgs::Lane getNewWaypoints() const
  {
    return updated_waypoints_;
  }

  autoware_msgs::Lane getTemporalWaypoints() const
  {
    return temporal_waypoints_;
  }

  bool getSetPath() const
  {
    return set_path_;
  }

  double getCurrentVelocity() const
  {
    return current_vel_;
  }

  int getPrevWaypointsSize() const
  {
    return original_waypoints_.waypoints.size();
  }

  int getNewWaypointsSize() const
  {
    return updated_waypoints_.waypoints.size();
  }
};

#endif  // WAYPOINT_PLANNER_VELOCITY_SET_VELOCITY_SET_PATH_H
