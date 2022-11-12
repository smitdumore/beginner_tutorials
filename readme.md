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
    - colcon


### Steps to install

 In a terminal navigate to your ros2 workspace src folder and type the following:
```
git clone https://github.com/smitdumore/beginner_tutorials.git
git checkout ros_pub_sub
cd ..
colcon build --packages-select cpp_pubsub
. install/setup.bash
```
### Steps to run
 
 In a terminal type
 ```
 . install/setup.bash
 ros2 run cpp_pubsub talker
 ```

 In a new terminal type
 ```
 . install/setup.bash
 ros2 run cpp_pubsub listener
 ```
