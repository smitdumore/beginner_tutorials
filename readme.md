[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# beginner_tutorials
---

### Description
 This repository contains a simple publisher and subscriber node


### Dependencies
- ROS version:
    - Humble Hawksbill
- Packages:
    - std_msgs
    - roscpp
    - ament

### Steps to build

 In a terminal navigate to your ros2 workspace and type the following:
```
cd <your_ros2_ws>/src
git clone https://github.com/smitdumore/beginner_tutorials.git
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select beginner_tutorials
. install/setup.bash
```

### Steps to run
## Steps to run publisher
 
 In a terminal type
 ```
 cd <your_ros2_ws>
 . install/setup.bash
 ros2 run beginner_tutorials talker
 ```

## Steps to run subcriber
 In a new terminal type
 ```
 cd <your_ros2_ws>
 . install/setup.bash
 ros2 run beginner_tutorials listener
 ```
