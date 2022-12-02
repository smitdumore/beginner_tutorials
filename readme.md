[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# beginner_tutorials
---

### Description
 This repository contains a simple publisher and subscriber node and ros client-server

 This repo uses a service call to edit a string that is being advertised by a publisher.

 This repo uses ROS inbuilt service file AddTwoInts.srv 


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

 ## Steps to run service node

 In a new terminal type
 ```
 cd <your_ros2_ws>
 . install/setup.bash
 ros2 run beginner_tutorials service <arg1> <arg2>
 ```
 note: arg1 and arg2 are integer values

 This will change the default sum value from zero to arg1 + arg2 and thus the publishing message will also reflect this change.
 
 ## Steps to launch both nodes

 In a new terminal type
 ```
 cd <ROS2_ws>/
 . install/setup.bash
 ros2 launch beginner_tutorials pubsub.launch.yaml hz:=20.0 bool_record_bag:=true
 ```

 ## Steps to call service manually
 In a new terminal type
 ```
 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts a:\ 10
 b:\ 10 
 ```

 ## inspecting transformation tf
 # 1. Using view_frames
 In a new terminal type
 ```
 ros2 run tf2_tools view_frames
 ```
 # 2. Using tf2 echo
 In a new terminal type
 ```
 ros2 run tf2_ros tf2_echo world talk
 ```

 ## Working with bag files
 # 1. Recording a bag file with all topics
 In a new terminal type
 ```
 ros2 bag record -a -o my_bag 
 ```
 # disable recording by pressing Ctrl + c 

 # 2. Inspecting bag file 
 ```
 ros2 bag info my_bag
 ```

 # 3. Playing bag files
 ```
 ros2 bag play my_bag
 ```

 # 4. Recording bag file with launch file
 ```
 ros2 launch beginner_tutorials pubsub.launch.yaml hz:=20.0 bool_record_bag:=true
 ```

 ## Playing back the bag file and Listener node
 In a terminal run the bag file continously
 ```
 ros2 bag play my_bag --loop
 ```
 In a new terminal run the Listener
 ```
 ros2 run beginner_tutorials listener
 ```

 ## Running ROS test
 # first complile the code
 ```
 colcon build --packages-select beginner_tutorials
 ```

 # run tests
 ```
 colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
 ```

 # check return status
 ```
 colcon test-result --test-result-base build/beginner_tutorials
 echo $?
 ```
