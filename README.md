# ROS2_AUTODESK_DIFF_ROBOT

## BUILD
```
cd ~/ros2_autodesk_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## STEP 1: Creating a workspace
```
mkdir -p ~/ros2_autodesk_ws/src
cd ~/ros2_autodesk_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create robot_description --build-type ament_cmake
```

## STEP 2: Create an empty world
Create a folder worlds with file world.sdf<br />
Add worlds to CMakeLists.txt<br />
add a .gitignore<br />
[BUILD](#build)<br />
gz sim world.sdf<br />

## STEP 3: Launch Gazebo with ros2
Create a folder launch with file world.launch.sdf<br />
Add launch to CMakeLists.txt<br />
[BUILD](#build)<br />
ros2 launch robot_description world.launch.py<br />