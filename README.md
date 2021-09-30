# EiT ROS packages
## Nodes for image services
### Server
The server provides the service `/get_image` of type sensor_msgs/Image. The request is of type Empty (no input).

## Install UR ROS Driver
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
