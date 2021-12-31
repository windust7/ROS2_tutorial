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
      * ㄴCMakeLists.txt
      * ㄴpackage.xml
      * ㄴinclude
      * ㄴsrc
      * ㄴmsg
        * ㄴ[UniformCircularVel.msg](https://github.com/windust7/ROS_tutorial/blob/main/UniformCircularVel.msg)
    * ㄴmy_first_ros2_pkg
      * ㄴpackage.xml
      * ㄴsetup.cfg
      * ㄴsetup.py
      * ㄴlaunch
      * ㄴresource
      * ㄴtest
      * ㄴmy_first_ros2_pkg
        * ㄴ __init__.py
        * ㄴ [cmd_vel_publisher.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_vel_publisher.py)
        * ㄴ [cmd_vel_subscriber.py](https://github.com/windust7/ROS_tutorial/blob/main/cmd_vel_subscriber.py)

### Requirements
Ubuntu 18.04, ROS2 dashing version
to see results, follow below at your workspace:

```
ros2 run turtlesim turtlesim_node
ros2 run my_first_ros2_pkg cmd_vel_publisher -r 2 -v 2 -c 1
ros2 run my_first_ros2_pkg cmd_vel_subscriber 
```
