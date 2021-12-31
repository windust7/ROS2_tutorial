# ROS_tutorial

## First Example
### Directory
* (your workspace)
  * ㄴbuild
  * ㄴinstall
  * ㄴlog
  * ㄴsrc
    * ㄴbuild
    * ㄴinstall
    * ㄴlog
    * ㄴmsg_interface_example
      * ㄴ[CMakeLists.txt](https://github.com/windust7/ROS_tutorial/blob/main/CMakeLists(for%20msg%20interface).txt)
      * ㄴ[package.xml](https://github.com/windust7/ROS_tutorial/blob/main/package(for%20msg%20interface).xml)
      * ㄴinclude
      * ㄴsrc
      * ㄴmsg
        * ㄴ[UniformCircularVel.msg](https://github.com/windust7/ROS_tutorial/blob/main/UniformCircularVel.msg)
    * ㄴmy_first_ros2_pkg
      * ㄴ[package.xml](https://github.com/windust7/ROS_tutorial/blob/main/package(for%20my_first_ros2_pkg).xml)
      * ㄴsetup.cfg
      * ㄴ[setup.py](https://github.com/windust7/ROS_tutorial/blob/main/setup(for%20my_first_ros2_pkg).py)
      * ㄴlaunch
        * ㄴ[cmd_to_twist.launch.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_to_twist.launch.py)
      * ㄴresource
      * ㄴtest
      * ㄴmy_first_ros2_pkg
        * ㄴ ```__init__.py```
        * ㄴ [cmd_vel_publisher.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_vel_publisher.py)
        * ㄴ [cmd_vel_subscriber.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_vel_subscriber.py)

### Requirements
Ubuntu 18.04, ROS2 dashing version

To see results, follow:
```
cd ~/(your workspace)/src
ros2 pkg create --build-type ament_cmake msg_interface_example
cd msg_interface_example
mkdir msg
```
Change [package.xml](https://github.com/windust7/ROS_tutorial/blob/main/package(for%20msg%20interface).xml) file

Change [CMakeLists.txt](https://github.com/windust7/ROS_tutorial/blob/main/CMakeLists(for%20msg%20interface).txt) file

Make [UniformCircularVel.msg](https://github.com/windust7/ROS_tutorial/blob/main/UniformCircularVel.msg) file
```
cd ~/(your workspace)/src/
ros2 pkg create my_first_ros2_pkg --build-type ament_python --dependencies rclpy std_msgs
```
Change [package.xml](https://github.com/windust7/ROS_tutorial/blob/main/package(for%20my_first_ros2_pkg).xml) file

Change [setup.py](https://github.com/windust7/ROS_tutorial/blob/main/setup(for%20my_first_ros2_pkg).py) file

Make [cmd_vel_publisher.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_vel_publisher.py) and [cmd_vel_subscriber.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_vel_subscriber.py) file

Make [cmd_to_twist.launch.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_to_twist.launch.py) file


```
cd ~/(your workspace)
colcon build --symlink-install --packages-select msg_interface_example
colcon build --symlink-install --packages-select my_first_ros2_pkg
. ~/(your workspace)/src/install/local_setup.bash
```

Follow below at different terminal window(r means radius, v means linear velocity, c means counterclockwise(1) or clockwise(0) direction:

```
ros2 run turtlesim turtlesim_node
ros2 run my_first_ros2_pkg cmd_vel_publisher -r 2 -v 2 -c 1
ros2 run my_first_ros2_pkg cmd_vel_subscriber 
```
To run launch file, follow below:

```
ros2 run turtlesim turtlesim_node
cd ~/(your workspace)/src/my_first_ros2_pkg/launch
ros2 launch cmd_to_twist.launch.py
```
![image](https://user-images.githubusercontent.com/62916482/147823667-9a4676db-a879-4948-ab2e-194bf2208e8a.png)

