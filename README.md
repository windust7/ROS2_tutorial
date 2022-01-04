# ROS_tutorial

## First Example
### Brief Description
![image](https://user-images.githubusercontent.com/62916482/148050456-924a0ab0-9360-4029-925f-c9c4e5a7deba.png)
![image](https://user-images.githubusercontent.com/62916482/148050467-6a40bf14-ede7-4356-9f53-fccba759aed9.png)


We give radius, linear velocity and direction of turtle's trajectory(uniform circular) to ["/rvd_publisher"](https://github.com/windust7/ROS_tutorial/blob/main/uniform_circle_turtle/uniform_circle_turtle/rvd_publisher.py) node.

Then ["/rvd_publisher"](https://github.com/windust7/ROS_tutorial/blob/main/uniform_circle_turtle/uniform_circle_turtle/rvd_publisher.py) node publishs "/uniform_circular_velocity" topic msg (["rvd_msg_example/msg/RVD"](https://github.com/windust7/ROS_tutorial/blob/main/rvd_msg_example/msg/RVD.msg) type) to ["/rvd_subscriber"](https://github.com/windust7/ROS_tutorial/blob/main/uniform_circle_turtle/uniform_circle_turtle/rvd_subscriber.py) node. This node transforms "/uniform_circular_velocity" topic msg (["rvd_msg_example/msg/RVD"](https://github.com/windust7/ROS_tutorial/blob/main/rvd_msg_example/msg/RVD.msg) type) to "/turtle1/cmd_vel" topic msg ("geometry_msgs/msg/Twist" type) and publishs to "/turtlesim" node.

[Launch file](https://github.com/windust7/ROS_tutorial/blob/main/uniform_circle_turtle/launch/rvd_to_twist.launch.py) includes ["/rvd_subscriber"](https://github.com/windust7/ROS_tutorial/blob/main/uniform_circle_turtle/uniform_circle_turtle/rvd_subscriber.py) node and turtlesim_node (turtlesim package). We give "/uniform_circular_velocity" topic msg (["rvd_msg_example/msg/RVD"](https://github.com/windust7/ROS_tutorial/blob/main/rvd_msg_example/msg/RVD.msg) type) to ["/rvd_subscriber"](https://github.com/windust7/ROS_tutorial/blob/main/uniform_circle_turtle/uniform_circle_turtle/rvd_subscriber.py) node.

### Requirements
Ubuntu 18.04, ROS2 dashing version

To see results, follow:
```
git clone https://github.com/windust7/ROS_tutorial
```
Move rvd_msg_example and uniform_circle_turtle folders to src folder(which is at your workspace)

```
cd ~/(your workspace)
colcon build --symlink-install --packages-select rvd_msg_example
colcon build --symlink-install --packages-select uniform_circle_turtle
. ~/(your workspace)/install/local_setup.bash
```
To run each nodes, follow below at different terminal window(r means radius, v means velocity, c means counterclockwise(1) or clockwise direction(0)):
```
ros2 run turtlesim turtlesim_node
ros2 run uniform_circle_turtle rvd_publisher -r 2 -v 2 -c 1
ros2 run uniform_circle_turtle rvd_subscriber 
```
To run launch file, follow below at different terminal window:

```
ros2 launch uniform_circle_turtle rvd_to_twist.launch.py
ros2 topic pub --once /uniform_circular_velocity rvd_msg_example/msg/RVD "{ccwdirection: False, radius: 1.5, lin_vel: 5.0}"
```

