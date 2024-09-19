#!/bin/bash
radius=600

while [ $radius -ge 8 ]; do
    echo $radius
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: $radius.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3600.0}}" &&
    radius=$((radius / 10 * 9))
done

