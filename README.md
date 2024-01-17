# Copley ROS Driver
The driver is used to send motor states and receive position command from client.

## Requirement
1. ROS Humble

## Run the package
1. After requirement installation and workspace setup, clone and build the package.

2. Run the driver as root:
```
cd $<workspace>
sudo -s
source /opt/ros/<ros_version>/setup.bash
source install/setup.bash
ros2 run copley_ros_driver copley_driver
```

3. Send an desired position once(optional, desired position unit as radian):
```
cd $<workspace>
sudo -s
source /opt/ros/<ros_version>/setup.bash
source install/setup.bash
ros2 topic pub --once /copley_desired_pos sensor_msgs/msg/JointState "{name: ['cmd'], position: [1.0], velocity: [0.0], effort: [0.0]}"
```
