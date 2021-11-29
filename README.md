# Obstacle avoidance robot simulation
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Create Catkin workspace:
From any desired directory
```
mkdir ws/src
cd ws/src
```

## Download TurtleBot simulation files:
```
sudo apt-get install ros-${ROS_DISTRO}-turtlebot3-gazebo
```

## Run simulation:
```
roslaunch obstacle_avoidance obstacle_avoidance.launch
```

## Run simulation with ROS Bag recording:
```
roslaunch obstacle_avoidance obstacle_avoidance.launch rosbag_record:=true
```

To play the bag file:
```
rosbag play "path-to-bag-file"
```

## Example Bag file:
Bag file [link]()


## Code Formatting check 
### Cpplint:
```
cd src/beginner_tutorials
cpplint $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/") > results/cpplintoutput.txt
```
### Cppcheck:
```
cppcheck --enable=all --std=c++11 --language=c++ -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/") > results/cppcheckoutput.txt
```