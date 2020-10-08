A package to bridge actions between ROS1 and ROS2. 

**NOTE:**   
- Currently supports forwarding goals from ROS1 (noetic) action client to ROS2 (foxy) action server
- As an example, implemented interfaces for the action bridge for FibonacciAction   
  and FollowJointTrajectoryAction  

**Prerequisites:**  

(*rosdep does not work properly with mixed ROS1 and ROS2 dependencies*)

```
sudo apt install ros-noetic-actionlib ros-noetic-actionlib-tutorials ros-noetic-control-msgs ros-noetic-roscpp ros-foxy-control-msgs ros-foxy-rclcpp ros-foxy-rclcpp-action ros-foxy-action-tutorials-interfaces
```

**How to build:**  

Clone the repository in the `src` folder of your ROS2 workspace.
```
git clone git@github.com:ipa-hsd/action_bridge.git -b foxy-devel
```

Since `action_bridge` package depends on both ROS1 and ROS2, source both workspaces.
```
source /opt/ros/noetic/local_setup.bash
source /opt/ros/foxy/local_setup.bash
colcon build
```
Now you are ready to run the `action_bridge`!  
Source this workspace to use the executeables built in the previous step. 
```
source <path-to-workspace>/install/local_setup.bash
```
4 example executables are available:
- `action_bridge_fibonacci_1_2`
- `action_bridge_fibonacci_2_1`
- `action_bridge_follow_joint_trajectory_1_2` and
- `action_bridge_follow_joint_trajectory_2_1`
You can start one of these nodes in the following manner:
```
ros2 run action_bridge action_bridge_fibonacci_1_2
```
OR
```
ros2 run action_bridge action_bridge_follow_joint_trajectory_1_2
```









