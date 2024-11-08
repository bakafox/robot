xacro robot.xacro > generated_from_xarco.urdf &&
ros2 launch display.launch.py \
    jspg:=True \
    rviz:=urdf_config.rviz \
    urdf:=generated_from_xarco.urdf &&
rm generated_from_xarco.urdf
