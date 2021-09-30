# EiT ROS packages
## Nodes for image services
### Server
The server provides the service `/get_image` of type sensor_msgs/Image. The request is of type Empty (no input).

## UR ROS Driver
### Installation
```
# go to catkin workspace
cd catkin_ws

# clone the driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# activate the workspace (ie: source it)
source devel/setup.bash
```
### Running

#### On the ROS PC
Run calibration of the robot

`roslaunch ur_calibration calibration_correction.launch   robot_ip:=192.168.1.20 target_filename:="${HOME}/ur10e_calibration.yaml"`

Run the driver

`roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.1.20 kinematics_config:=${HOME}/ur10e_calibration.yaml`

#### On the robot
Check if the ROS PC IP is correct in `Installation -> URCaps -> External Control`. If not either load it from `Open -> Installation` or create a new one.
