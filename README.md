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

# GOAL 1: Render model in Gazebo

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

# GOAL 2: Move model in Gazebo and rviz with ROS
**Major Changes are in this Goal**

## STEP 1: 
- Add a Base into xacro file(2D ground reference)
```
<!-- Base footprint (planar frame) -->
	<joint name="base_footprint_joint" type="fixed">
		<origin xyz="0 0 0.05" rpy="0 0 0" />
		<parent link="base_footprint" />
		<child link="base_link" />
	</joint>
```
- Add controller plugin and for friction in same file at last before </robot> tag
```
    <gazebo>
		<plugin filename="gz-sim-diff-drive-system"
			name="gz::sim::systems::DiffDrive">
			<left_joint>wheel_left_joint</left_joint>
			<right_joint>wheel_right_joint</right_joint>
			<wheel_separation>0.20</wheel_separation>
			<wheel_radius>0.05</wheel_radius>
			<topic>/cmd_vel</topic>
			<odom_topic>/odom</odom_topic>
			<tf_topic>/tf</tf_topic>
			<frame_id>odom</frame_id>
			<child_frame_id>base_link</child_frame_id>
			<odom_publish_frequency>30</odom_publish_frequency>
		</plugin>
		<plugin filename="gz-sim-joint-state-publisher-system"
			name="gz::sim::systems::JointStatePublisher">
			<topic>joint_states</topic>
			<joint_name>wheel_left_joint</joint_name>
			<joint_name>wheel_right_joint</joint_name>
		</plugin>
	</gazebo>
	<gazebo reference="left_wheel_1">
		<mu1>1.0</mu1>
		<mu2>1.0</mu2>
	</gazebo>
	<gazebo reference="right_wheel_1">
		<mu1>1.0</mu1>
		<mu2>1.0</mu2>
	</gazebo>
	<gazebo reference="caster_1">
		<mu1>0.0</mu1>
		<mu2>0.0</mu2>
	</gazebo>
```
## STEP 2: Test movement now
- [BUILD](#build)<br />
- Terminal 1: ros2 launc robot_description gazebo.launch.py<br />
- Terminal 2: ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"<br />
- Model should rotate

## STEP 3: yaml, rviz changes (refer to github files)
- Create a new file ros2_controllers.yaml inside config folder
- Make changes in ros_gz_bridge.yaml 
- display.rviz -> change Fixed Frame: base_link to odom

## STEP 4: Synmc Gazebo and RVIZ with ROS2
- First install teleop to control the model ```sudo apt install ros-jazzy-teleop-twist-keyboard```<br />
- Create a file in launch folder(refer file for changes)<br />
- [BUILD](#build)
- Terminal 1: ros2 launc robot_description sim_rviz.launch.py<br />
- Terminal 2: ros2 run teleop_twist_keyboard teleop_twist_keyboard<br />
- You should be able to type commands and model will be moving