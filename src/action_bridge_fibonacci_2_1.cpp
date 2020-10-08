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

#include <action_bridge/action_bridge_2_1.hpp>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <actionlib_tutorials/FibonacciAction.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include <action_tutorials_interfaces/action/fibonacci.hpp>

using FibonacciActionBridge = ActionBridge_2_1<actionlib_tutorials::FibonacciAction,
                                               action_tutorials_interfaces::action::Fibonacci>;

template <>
void FibonacciActionBridge::translate_goal_2_to_1(const ROS2Goal &goal2, ROS1Goal &goal1)
{
  goal1.order = goal2.order;
}

template <>
void FibonacciActionBridge::translate_result_1_to_2(
    ROS2Result &result2,
    const ROS1Result &result1)
{
  result2.sequence = result1.sequence;
}

template <>
void FibonacciActionBridge::translate_feedback_1_to_2(
    ROS2Feedback &feedback2,
    const ROS1Feedback &feedback1)
{
  feedback2.partial_sequence = feedback1.sequence;
}

int main(int argc, char *argv[])
{
  return FibonacciActionBridge::main("fibonacci", argc, argv);
}
