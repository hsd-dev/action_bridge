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

#ifndef ACTION_BRIDGE__ACTION_BRIDGE_HPP_
#define ACTION_BRIDGE__ACTION_BRIDGE_HPP_

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

template <class ROS1_T, class ROS2_T>
class ActionBridge
{
public:
  using ROS1GoalHandle = typename actionlib::ActionServer<ROS1_T>::GoalHandle;
  using ROS2ServerGoalHandle = typename rclcpp_action::ServerGoalHandle<ROS2_T>;
  using ROS2Goal = typename ROS2_T::Goal;
  ActionBridge(
      ros::NodeHandle ros1_node,
      rclcpp::Node::SharedPtr ros2_node,
      const std::string action_name)
      : ros1_node_(ros1_node), ros2_node_(ros2_node),
        server_1_(ros1_node, action_name,
                  std::bind(&ActionBridge::goal_cb, this, std::placeholders::_1),
                  std::bind(&ActionBridge::cancel_cb, this, std::placeholders::_1),
                  false),
        client_1_(ros1_node, action_name)
  {
    server_1_.start();
    client_2_ = rclcpp_action::create_client<ROS2_T>(ros2_node, action_name);

    server_2_ = rclcpp_action::create_server<ROS2_T>(ros2_node_->get_node_base_interface(),
                                                     ros2_node_->get_node_clock_interface(),
                                                     ros2_node_->get_node_logging_interface(),
                                                     ros2_node_->get_node_waitables_interface(),
                                                     action_name,
                                                     std::bind(&ActionBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                     std::bind(&ActionBridge::handle_cancel, this, std::placeholders::_1),
                                                     std::bind(&ActionBridge::handle_accepted, this, std::placeholders::_1));
  }

  // ROS1 callbacks
  void cancel_cb(ROS1GoalHandle gh1)
  {
    // try to find goal and cancel it
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = goals_.find(gh1.getGoalID().id);
    if (it != goals_.end())
    {
      std::thread([handler = it->second]() mutable {
        handler->cancel();
      })
          .detach();
    }
  }
  void goal_cb(ROS1GoalHandle gh1)
  {
    const std::string goal_id = gh1.getGoalID().id;

    // create a new handler for the goal
    std::shared_ptr<GoalHandler21> handler;
    handler.reset(new GoalHandler21(gh1, client_2_));
    std::lock_guard<std::mutex> lock(mutex_);
    goals_.insert(std::make_pair(goal_id, handler));

    RCLCPP_INFO(ros2_node_->get_logger(), "Sending goal");
    std::thread([handler, goal_id, this]() mutable {
      // execute the goal remotely
      handler->handle();

      // clean-up
      std::lock_guard<std::mutex> lock(mutex_);
      goals_.erase(goal_id);
    })
        .detach();
  }

  //ROS2 callbacks
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const ROS2Goal> goal)
  {
    (void)uuid;
    // return rclcpp_action::GoalResponse::ACCEPTED;
  }

  rclcpp_action::CancelResponse handle_cancel(
      std::shared_ptr<ROS2ServerGoalHandle> gh2)
  {
    (void)gh2;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<ROS2ServerGoalHandle> gh2)
  {
  }

  static int main(const std::string &action_name, int argc, char *argv[])
  {
    std::string node_name = "action_bridge_" + action_name;
    std::replace(node_name.begin(), node_name.end(), '/', '_');
    // ROS 1 node
    ros::init(argc, argv, node_name);
    ros::NodeHandle ros1_node;

    // ROS 2 node
    rclcpp::init(argc, argv);
    auto ros2_node = rclcpp::Node::make_shared(node_name);

    ActionBridge<ROS1_T, ROS2_T> action_bridge(ros1_node, ros2_node, action_name);

    // // ROS 1 asynchronous spinner
    ros::AsyncSpinner async_spinner(0);
    async_spinner.start();

    rclcpp::spin(ros2_node);
    ros::shutdown();
    return 0;
  }

private:
  using ROS1Server = typename actionlib::ActionServer<ROS1_T>;
  using ROS1Client = typename actionlib::ActionClient<ROS1_T>;
  using ROS1Goal = typename actionlib::ActionServer<ROS1_T>::Goal;
  using ROS1Feedback = typename actionlib::ActionServer<ROS1_T>::Feedback;
  using ROS1Result = typename actionlib::ActionServer<ROS1_T>::Result;

  using ROS2Feedback = typename ROS2_T::Feedback;
  using ROS2Result = typename ROS2_T::Result;
  using ROS2ClientGoalHandle = typename rclcpp_action::ClientGoalHandle<ROS2_T>::SharedPtr;
  using ROS2ClientSharedPtr = typename rclcpp_action::Client<ROS2_T>::SharedPtr;
  using ROS2ServerSharedPtr = typename rclcpp_action::Server<ROS2_T>::SharedPtr;

  using ROS2SendGoalOptions = typename rclcpp_action::Client<ROS2_T>::SendGoalOptions;

  class GoalHandler21
  {
  public:
    void cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      canceled_ = true;
      if (gh2_)
      { // cancel goal if possible
        auto fut = client_2_->async_cancel_goal(gh2_);
      }
    }
    void handle()
    {
      auto goal1 = gh1_.getGoal();
      ROS2Goal goal2;
      translate_goal_1_to_2(*gh1_.getGoal(), goal2);

      if (!client_2_->wait_for_action_server(std::chrono::seconds(1)))
      {
        std::cout << "Action server not available after waiting" << std::endl;
        gh1_.setRejected();
        return;
      }

      //Changes as per Dashing
      auto send_goal_ops = ROS2SendGoalOptions();
      send_goal_ops.goal_response_callback =
          [this](auto gh2_future) mutable {
            auto goal_handle = gh2_future.get();
            if (!goal_handle)
            {
              gh1_.setRejected(); // goal was not accepted by remote server
              return;
            }

            gh1_.setAccepted();

            {
              std::lock_guard<std::mutex> lock(mutex_);
              gh2_ = goal_handle;

              if (canceled_)
              { // cancel was called in between
                auto fut = client_2_->async_cancel_goal(gh2_);
              }
            }
          };

      send_goal_ops.feedback_callback =
          [this](ROS2ClientGoalHandle, auto feedback2) mutable {
            ROS1Feedback feedback1;
            translate_feedback_2_to_1(feedback1, *feedback2);
            gh1_.publishFeedback(feedback1);
          };

      // send goal to ROS2 server, set-up feedback
      auto gh2_future = client_2_->async_send_goal(goal2, send_goal_ops);

      auto future_result = client_2_->async_get_result(gh2_future.get());
      auto res2 = future_result.get();

      ROS1Result res1;
      translate_result_2_to_1(res1, *(res2.result));

      std::lock_guard<std::mutex> lock(mutex_);
      if (res2.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        gh1_.setSucceeded(res1);
      }
      else if (res2.code == rclcpp_action::ResultCode::CANCELED)
      {
        gh1_.setCanceled(res1);
      }
      else
      {
        gh1_.setAborted(res1);
      }
    }

    GoalHandler21(ROS1GoalHandle &gh1, ROS2ClientSharedPtr &client)
        : gh1_(gh1), gh2_(nullptr), client_2_(client), canceled_(false) {}

  private:
    ROS1GoalHandle gh1_;
    ROS2ClientGoalHandle gh2_;
    ROS2ClientSharedPtr client_2_;
    bool canceled_; // cancel was called
    std::mutex mutex_;
  };

  class GoalHandler12
  {
  public:
    void cancel()
    {
    }
    void handle()
    {
    }

    GoalHandler12(ROS2ServerGoalHandle &gh2, ROS1Client &client)
        : gh2_(gh2), gh1_(nullptr), client_1_(client), canceled_(false) {}

  private:
    ROS1GoalHandle gh1_;
    ROS2ServerGoalHandle gh2_;
    ROS1Client client_1_;
    bool canceled_; // cancel was called
    std::mutex mutex_;
  };

  ros::NodeHandle ros1_node_;
  rclcpp::Node::SharedPtr ros2_node_;

  ROS1Server server_1_;
  ROS1Client client_1_;
  ROS2ClientSharedPtr client_2_;
  ROS2ServerSharedPtr server_2_;

  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<GoalHandler21>> goals_;

  static void translate_goal_1_to_2(const ROS1Goal &, ROS2Goal &);
  static void translate_result_2_to_1(ROS1Result &, const ROS2Result &);
  static void translate_feedback_2_to_1(ROS1Feedback &, const ROS2Feedback &);
};

#endif // ACTION_BRIDGE__ACTION_BRIDGE_HPP_
