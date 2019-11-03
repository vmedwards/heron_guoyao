# heron_guoyao
A repo to store the scripts for Heron

### Connect to Heron & Basic Command
(1) To connect with Heron, first use your pc to connect the WIFI from Heron: “PEN09 Base Station”. Then open the terminal and run:
```cpp
ssh administrator@192.168.131.1
```
(2) Check available topics or nodes
```cpp
rostopic list
rosnode list
```
(3) Echo particular topic & Kill node on Heron
```cpp
rostopic echo TOPIC_YOU_NEED
rosnode kill NODE_NEED_KILL
```
(4) Turn off the lights
```cpp
rostopic pub /disable_lights std_msgs/Bool "data: true"
```
