# ROS2_AUTODESK_DIFF_ROBOT

## Installation
- ROS2 JAZZY: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- GAZEBO HARMONIC: https://gazebosim.org/docs/harmonic/install_ubuntu
- ROS–Gazebo bridge: ``` sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz ```
- OTHERS: ``` sudo apt install python3-colcon-common-extensions ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ```
- CHECKING: ``` sudo apt install liburdfdom-tools ```


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
* Create a folder worlds with file world.sdf<br />
* Add worlds to CMakeLists.txt<br />
* add a .gitignore<br />
* [BUILD](#build)<br />
* gz sim world.sdf<br />

## STEP 3: Launch Gazebo with ros2
- Create a folder launch with file world.launch.sdf<br />
- Add launch to CMakeLists.txt<br />
- [BUILD](#build)<br />
- ros2 launch robot_description world.launch.py<br />

## STEP 4: Creating a differential robot in Fusion 360
- Create a model in fusion 360<br />
- Add an external plugin ACDC4Robot<br />
- **Folder name for Fusion 360 and Package name for workspace should be same (robot_description)**
- export the model with ACDC4Robot, it gives (meshes, config, launc, xm, cmakelist)

## STEP 4: Import Fusion 360 files into workspace
- Move config, urdf, meshes, launch folder to robot_description folder<br />
- Add urdf config meshes into CMakeLists.txt<br/>
- Add these in package.xml inside description
```
    <depend>xacro</depend>
    <depend>rviz2</depend>
    <depend>robot_state_publisher</depend>
    <depend>joint_state_publisher</depend>
    <depend>joint_state_publisher_gui</depend>
```
- [BUILD](#build)<br />
- ros2 launch robot_description world.launch.py<br />
- ros2 launch robot_description gazebo.launch.py<br />
- ros2 launch robot_description rviz.launch.py<br />