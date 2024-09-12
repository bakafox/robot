#!/bin/bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 300.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -40.0}}" && sleep 1 &&
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 340.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 4200.0}}" && sleep 1 &&
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 300.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -14.0}}" && sleep 1 &&
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 300.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -2900.0}}" && sleep 1 &&
echo "восмербка успешно нарисвоана..." && sleep 4 &&
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 360.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3600.0}}"
