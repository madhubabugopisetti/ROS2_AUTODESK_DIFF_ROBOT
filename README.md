# ROS2_AUTODESK_DIFF_ROBOT

## STEP 0: Creating a workspace
```
mkdir -p ~/ros2_autodesk_ws/src
cd ~/ros2_autodesk_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create robot_description --build-type ament_cmake
```

## BUILD
```
cd ~/ros2_autodesk_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## STEP 1: ### Create an empty world
Create a folder worlds with file world.sdf
Add worlds to CMakeLists.txt
add a .gitignore
[Link Text](#build)
gz sim world.sdf

## STEP 2: ### Launch Gazebo with ros2
Create a folder launch with file world.launch.sdf
Add launch to CMakeLists.txt
[Link Text](#build)
ros2 launch robot_description world.launch.py