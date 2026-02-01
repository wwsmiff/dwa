### Setup

#### Setup the necessary files
```
git clone https://github.com/wwsmiff/dwa
cd src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble
cd ~/dwa
colcon build
source ./install/setup.bash
```

### Run the necessary ros nodes
Launch gazebo with turtlebot3
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Launch slam_toolbox
```
ros2 run slam_toolbox async_slam_toolbox_node
```

Launch rviz2
```
ros2 run rviz2 rviz2
```

Add the necessary visualization components into rviz2
```
Add LaserScan and set topic to /scan
Add Map and set topic to /map
Add TF
```

Launch the dwa_node
```
ros2 run dwa_navigation dwa_node
```

Now just just click and select a goal pose in rviz2
