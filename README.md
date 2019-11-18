[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

# Turtlebot Walker

ROS node that controls the movement of a Turtlebot in a simulated Gazebo environment.

The walker algorithm moves the robot straight forward until it detects an obstacle with its laser scan sensor. When an obstacle is detected, the robot will turn in place until the obstacle is no longer in front of the robot.

## License

BSD License
Copyright 2019 Ryan Cunningham

```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

```

## Dependencies

### Install ROS

Install ROS Kinetic on Ubuntu system (full installation guide at [ROS installation page](https://wiki.ros.org/kinetic/installation/Ubuntu)):

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

### Create Catkin Workspace

Create a Catkin workspace if one does not already exist:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
```

## Installation

Clone repo to Catkin Workspace:

```
cd ~/catkin_ws/src/
git clone https://github.com/cunninghamr/turtlebot_walker.git
```

## Build

Build the walker node:

```
cd ~/catkin_ws/
catkin_make
```

## Run

Setup catkin workspace:

```
cd ~/catkin_ws/
source ./devel/setup.bash
```

Start roscore (if not already started):

```
roscore
```

### Launch File

Start the walker node along with Gazebo by entering the following command:

```
roslaunch turtlebot_walker walker.launch
```

### Rosbag

#### Recording

The launch file also accepts an optional boolean argument, 'record', which will record and save a bag file (named bagout.bag) of the topics used by the nodes. NOTE: The launch file does not have recording enabled by default.

To run the node and record a bag file, run the following command:

```
ROS_HOME=`pwd` roslaunch turtlebot_walker walker.launch record:="true"
```

Use `ctrl-C` to stop the node, and inspect the recorded bag file using the following command:

```
rosbag info bagout.bag
```

#### Playback

To play back the recorded bag, start the node without Gazebo using the following command:

```
rosrun turtlebot_walker main
```

In a new terminal, enter the following command to play the bag file:

```
rosbag play bagout.bag
```

The command will output the bag status, and the node should emit log messages similar to:

```
[ WARN] [1574093997.786296870, 541.261000000]: Laser scan value is NaN
[ INFO] [1574093997.786322719, 541.261000000]: Obstacle detected
[ WARN] [1574093997.886747488, 541.361000000]: Laser scan value is NaN
[ INFO] [1574093997.886841500, 541.361000000]: Obstacle detected
[ INFO] [1574093997.986539799, 541.461000000]: Obstacle detected
[ INFO] [1574093998.087307588, 541.562000000]: Obstacle detected
[ INFO] [1574093998.187466312, 541.662000000]: Obstacle detected
[ INFO] [1574093998.287471111, 541.761000000]: Obstacle detected
```
