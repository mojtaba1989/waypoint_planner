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

#include <waypoint_planner/velocity_set/libvelocity_set.h>

#include <map>
#include <vector>

// extract edge points from zebra zone
std::vector<geometry_msgs::Point> removeNeedlessPoints(std::vector<geometry_msgs::Point> &area_points)
{
  area_points.push_back(area_points.front());
  std::map<double, int> length_index;
  for (unsigned int i = 0; i < area_points.size() - 1; i++)
    length_index[calcSquareOfLength(area_points[i], area_points[i + 1])] = i;

  std::vector<geometry_msgs::Point> new_points;
  auto it = length_index.end();
  int first = (--it)->second;
  int second = (--it)->second;
  new_points.push_back(area_points[first]);
  new_points.push_back(area_points[first + 1]);
  new_points.push_back(area_points[second]);
  new_points.push_back(area_points[second + 1]);

  return new_points;
}

void CrossWalk::crossWalkCallback(const vector_map::CrossWalkArray &msg)
{
  crosswalk_ = msg;

  loaded_crosswalk = true;
  if (loaded_crosswalk && loaded_area && loaded_line && loaded_point)
  {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }
}

void CrossWalk::areaCallback(const vector_map::AreaArray &msg)
{
  area_ = msg;

  loaded_area = true;
  if (loaded_crosswalk && loaded_area && loaded_line && loaded_point)
  {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }
}

void CrossWalk::lineCallback(const vector_map::LineArray &msg)
{
  line_ = msg;

  loaded_line = true;
  if (loaded_crosswalk && loaded_area && loaded_line && loaded_point)
  {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }
}

void CrossWalk::pointCallback(const vector_map::PointArray &msg)
{
  point_ = msg;

  loaded_point = true;
  if (loaded_crosswalk && loaded_area && loaded_line && loaded_point)
  {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }
}

geometry_msgs::Point CrossWalk::getPoint(const int &pid) const
{
  geometry_msgs::Point point;
  for (const auto &p : point_.data)
  {
    if (p.pid == pid)
    {
      point.x = p.ly;
      point.y = p.bx;
      point.z = p.h;
      return point;
    }
  }

  ROS_ERROR("can't find a point of pid %d", pid);
  return point;
}

geometry_msgs::Point CrossWalk::calcCenterofGravity(const int &aid) const
{
  int search_lid = -1;
  for (const auto &area : area_.data)
    if (area.aid == aid)
    {
      search_lid = area.slid;
      break;
    }

  std::vector<geometry_msgs::Point> area_points;
  while (search_lid)
  {
    for (const auto &line : line_.data)
    {
      if (line.lid == search_lid)
      {
        area_points.push_back(getPoint(line.bpid));
        search_lid = line.flid;
      }
    }
  }

  geometry_msgs::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  if (area_points.size() > 4)
  {
    std::vector<geometry_msgs::Point> filterd_points = removeNeedlessPoints(area_points);
    for (const auto &p : filterd_points)
    {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
  }
  else
  {
    for (const auto &p : area_points)
    {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
  }

  point.x /= 4;
  point.y /= 4;
  point.z /= 4;
  return point;
}

double CrossWalk::calcCrossWalkWidth(const int &aid) const
{
  int search_lid = -1;
  for (const auto &area : area_.data)
    if (area.aid == aid)
    {
      search_lid = area.slid;
      break;
    }

  std::vector<geometry_msgs::Point> area_points;
  while (search_lid)
  {
    for (const auto &line : line_.data)
    {
      if (line.lid == search_lid)
      {
        area_points.push_back(getPoint(line.bpid));
        search_lid = line.flid;
      }
    }
  }

  area_points.push_back(area_points.front());
  double max_length = calcSquareOfLength(area_points[0], area_points[1]);
  for (unsigned int i = 1; i < area_points.size() - 1; i++)
  {
    if (calcSquareOfLength(area_points[i], area_points[i + 1]) > max_length)
      max_length = calcSquareOfLength(area_points[i], area_points[i + 1]);
  }

  return sqrt(max_length);
}

// count the number of crosswalks
int CrossWalk::countAreaSize() const
{
  int count = 0;
  for (const auto &x : crosswalk_.data)
    if (x.type == 0)  // type:0 -> outer frame of crosswalks
      count++;

  return count;
}

void CrossWalk::getAID(std::unordered_map<int, std::vector<int>> &bdid2aid_map) const
{
  for (const auto &x : crosswalk_.data)
    if (x.type == 1)
    {                                         // if it is zebra
      bdid2aid_map[x.bdid].push_back(x.aid);  // save area id
    }
}

void CrossWalk::calcDetectionArea(const std::unordered_map<int, std::vector<int>> &bdid2aid_map)
{
  for (const auto &crosswalk_aids : bdid2aid_map)
  {
    int bdid = crosswalk_aids.first;
    double width = 0.0;
    for (const auto &aid : crosswalk_aids.second)
    {
      detection_points_[bdid].points.push_back(calcCenterofGravity(aid));
      width += calcCrossWalkWidth(aid);
    }
    width /= crosswalk_aids.second.size();
    detection_points_[bdid].width = width;
  }
}

void CrossWalk::calcCenterPoints()
{
  for (const auto &i : bdID_)
  {
    geometry_msgs::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = 0.0;
    for (const auto &p : detection_points_[i].points)
    {
      center.x += p.x;
      center.y += p.y;
      center.z += p.z;
    }
    center.x /= detection_points_[i].points.size();
    center.y /= detection_points_[i].points.size();
    center.z /= detection_points_[i].points.size();
    detection_points_[i].center = center;
  }
}

void CrossWalk::setCrossWalkPoints()
{
  // bdid2aid_map[BDID] has AIDs of its zebra zone
  std::unordered_map<int, std::vector<int>> bdid2aid_map;
  getAID(bdid2aid_map);

  // Save key values
  for (const auto &bdid2aid : bdid2aid_map)
    bdID_.push_back(bdid2aid.first);

  calcDetectionArea(bdid2aid_map);
  calcCenterPoints();

  ROS_INFO("Set cross walk detection points");
  set_points = true;
}

int CrossWalk::findClosestCrosswalk(const int closest_waypoint, const autoware_msgs::Lane &lane,
                                    const int search_distance)
{
  if (!set_points || closest_waypoint < 0)
    return -1;

  static constexpr double find_distance = 2.0 * 2.0;      // meter
  static constexpr double ignore_distance = 20.0 * 20.0;  // meter

  bool is_found = false;
  // keep the first crosswalks waypoint index if found
  int closest_crosswalk_waypoint_idx = -1;
  detection_crosswalk_id_ = -1;
  detection_crosswalk_array_.clear();  // for multiple

  // Find crosswalks within search distance
  for (int wp_idx = closest_waypoint;
        wp_idx < closest_waypoint + search_distance && wp_idx < static_cast<int>(lane.waypoints.size());
        wp_idx++)
  {
    geometry_msgs::Point waypoint = lane.waypoints[wp_idx].pose.pose.position;
    waypoint.z = 0.0;  // ignore Z axis
    for (const int &id : bdID_)
    {
      // ignore far crosswalk
      const auto& crosswalk = detection_points_.at(id);
      geometry_msgs::Point crosswalk_center = crosswalk.center;
      crosswalk_center.z = 0.0;
      if (calcSquareOfLength(crosswalk_center, waypoint) > ignore_distance)
        continue;

      for (auto p : crosswalk.points)
      {
        p.z = waypoint.z;
        if (calcSquareOfLength(p, waypoint) < find_distance)
        {
          if (!is_found)
          {
            closest_crosswalk_waypoint_idx = wp_idx;
            detection_crosswalk_id_ = id;
            is_found = true;
          }
          addDetectionCrossWalkIDs(id);
          if (!enable_multiple_crosswalk_detection_)
          {
            return closest_crosswalk_waypoint_idx;
          }
        }
      }
    }
  }

  return closest_crosswalk_waypoint_idx;
}

geometry_msgs::Point ObstaclePoints::getObstaclePoint(const EControl &kind) const
{
  geometry_msgs::Point point;

  if (kind == EControl::STOP || kind == EControl::STOPLINE)
  {
    for (const auto &p : stop_points_)
    {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
    point.x /= stop_points_.size();
    point.y /= stop_points_.size();
    point.z /= stop_points_.size();

    return point;
  }
  else  // kind == DECELERATE
  {
    for (const auto &p : decelerate_points_)
    {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
    point.x /= decelerate_points_.size();
    point.y /= decelerate_points_.size();
    point.z /= decelerate_points_.size();

    return point;
  }
}