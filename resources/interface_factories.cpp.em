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
@
@{
from ros1_bridge import camel_case_to_lower_case_underscore
}@

//need a find the proper path for the factories
#include "@(ros2_package_name)_factories.hpp"
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

// include builtin interfaces
#include <ros1_bridge/convert_builtin_interfaces.hpp>


#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
// include ROS 1 actions
@[for action in mapped_actions]@
#include <@(action["ros1_package"])/@(action["ros1_name"]).h>
hello
@[end for]@
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2 actions
@[for action in mapped_actions]@
#include <@(action["ros2_package"])/action/@(camel_case_to_lower_case_underscore(action["ros2_name"])).hpp>
@[end for]@


std::unique_ptr<ActionFactoryInterface>
get_action_factory_@(ros2_package_name)__@(interface_type)__@(interface.message_name)(const std::string & ros_id, const std::string & package_name, const std::string & action_name)
{
@[if not mapped_actions]@
  (void)ros_id;
  (void)package_name;
  (void)action_name;
@[end if]@
@[for action in mapped_actions]@
  if (
    (
      ros_id == "ros1" &&
      package_name == "@(action["ros1_package"])" &&
      action_name == "@(action["ros1_name"])"
    ) || (
      ros_id == "ros2" &&
      package_name == "@(action["ros2_package"])" &&
      action_name == "action/@(action["ros2_name"])"
    )
  ) {
    return std::unique_ptr<ActionFactoryInterface>(new ActionFactory<
      @(action["ros1_package"])::@(action["ros1_name"]),
      @(action["ros2_package"])::srv::@(action["ros2_name"])
    >);
  }
@[end for]@
  return nullptr;
}
@

@[for action in mapped_actions]@
@[  for frm, to in [("1", "2"), ("2", "1")]]@
@[    for type in ["Goal", "Result", "Feedback"]]@
template <>
void ActionFactory<
@(action["ros1_package"])::@(action["ros1_name"])Action,
@(action["ros2_package"])::action::@(action["ros2_name"])
>::translate_@(type.lower())_@(frm)_to_@(to)(
@[      if type == "Goal"]@
  const ROS@(frm)Goal &@(type.lower())@(frm), 
  ROS@(to)Goal &@(type.lower())@(to))
@[      else]@
  const ROS@(to)Goal &@(type.lower())@(to), 
  ROS@(frm)Goal &@(type.lower())@(frm))
@[      end if]@
{
@[      for field in action["fields"][type.lower()]]@
@[        if field["array"]]@
  @(type.lower())@(to).@(field["ros" + frm]["name"]).resize(@(type.lower())@(frm).@(field["ros" + to]["name"]).size());
  auto @(field["ros" + frm]["name"])@(frm)_it = @(type.lower())@(frm).@(field["ros" + frm]["name"]).begin();
  auto @(field["ros" + to]["name"])@(to)_it = @(type.lower())@(to).@(field["ros" + to]["name"]).begin();
  while (
    @(field["ros" + frm]["name"])@(frm)_it != @(type.lower())@(frm).@(field["ros" + frm]["name"]).end() &&
    @(field["ros" + to]["name"])@(to)_it != @(type.lower())@(to).@(field["ros" + to]["name"]).end()
  ) {
    auto & @(field["ros" + frm]["name"])@(frm) = *(@(field["ros" + frm]["name"])@(frm)_it++);
    auto & @(field["ros" + to]["name"])@(to) = *(@(field["ros" + to]["name"])@(to)_it++);
@[        else]@
  auto & @(field["ros" + frm]["name"])@(frm) = @(type.lower())@(frm).@(field["ros" + frm]["name"]);
  auto & @(field["ros" + to]["name"])@(to) = @(type.lower())@(to).@(field["ros" + to]["name"]);
@[        end if]@
@[      if field["basic"]]@
    @(field["ros" + to]["name"])@(to) = @(field["ros" + frm]["name"])@(frm);
@[      else]@
    Factory<@(field["ros" + frm]["cpptype"]),@(field["ros" + to]["cpptype"])>::convert_@(frm)_to_@(to)(@(field["ros" + frm]["name"])@(frm), @(field["ros" + to]["name"])@(to));
@[end if]@
@[        if field["array"]]@
  }
@[        end if]@

@[      end for]@
}

@[    end for]@

@[  end for]@

@[end for]@

                                        

