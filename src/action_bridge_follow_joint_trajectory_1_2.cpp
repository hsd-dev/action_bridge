// Copyright 2019 Fraunhofer IPA
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

#include <action_bridge/action_bridge_1_2.hpp>

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <control_msgs/FollowJointTrajectoryAction.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "control_msgs/action/follow_joint_trajectory.hpp"

template <typename T1, typename T2>
static void copy_point(const T1 &pt1, T2 &pt2)
{
  pt2.positions = pt1.positions;
  pt2.velocities = pt1.velocities;
  pt2.accelerations = pt1.accelerations;
}

template <typename T1, typename T2>
static void copy_tolerance(const T1 &tolerance1, T2 &tolerance2)
{
  tolerance2.name = tolerance1.name;
  tolerance2.position = tolerance1.position;
  tolerance2.velocity = tolerance1.velocity;
  tolerance2.acceleration = tolerance1.acceleration;
}

template <typename T1, typename T2>
static void copy_tolerances(const T1 &t1, T2 &t2)
{
  const size_t num = t1.size();
  t2.resize(num);
  for (size_t i = 0; i < num; ++i)
  {
    copy_tolerance(t1[i], t2[i]);
  }
}

static void copy_duration_1_to_2(const ros::Duration &duration1,
                                 builtin_interfaces::msg::Duration &duration2)
{
  duration2.sec = duration1.sec;
  duration2.nanosec = duration1.nsec;
}

using FollowJointTrajectoryActionBridge =
    ActionBridge_1_2<control_msgs::FollowJointTrajectoryAction,
                     control_msgs::action::FollowJointTrajectory>;

template <>
void FollowJointTrajectoryActionBridge::translate_goal_1_to_2(
    const ROS1Goal &goal1, ROS2Goal &goal2)
{
  goal2.trajectory.joint_names = goal1.trajectory.joint_names;
  const size_t num = goal1.trajectory.points.size();
  goal2.trajectory.points.resize(num);

  for (size_t i = 0; i < num; ++i)
  {
    copy_point(goal1.trajectory.points[i], goal2.trajectory.points[i]);
    copy_duration_1_to_2(goal1.trajectory.points[i].time_from_start,
                         goal2.trajectory.points[i].time_from_start);
  }

  copy_tolerances(goal1.path_tolerance, goal2.path_tolerance);
  copy_tolerances(goal1.goal_tolerance, goal2.goal_tolerance);

  copy_duration_1_to_2(goal1.goal_time_tolerance, goal2.goal_time_tolerance);
}

template <>
void FollowJointTrajectoryActionBridge::translate_result_2_to_1(
    ROS1Result &result1, const ROS2Result &result2)
{
  result1.error_code = result2.error_code;
  result1.error_string = result2.error_string;
}

template <>
void FollowJointTrajectoryActionBridge::translate_feedback_2_to_1(
    ROS1Feedback &feedback1, const ROS2Feedback &feedback2)
{
  feedback1.joint_names = feedback2.joint_names;
  copy_point(feedback2.desired, feedback1.desired);
  copy_point(feedback2.actual, feedback1.actual);
  copy_point(feedback2.error, feedback1.error);
}

int main(int argc, char *argv[])
{
  return FollowJointTrajectoryActionBridge::main("follow_joint_trajectory",
                                                 argc, argv);
}
