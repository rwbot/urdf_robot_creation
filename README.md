#  urdf_robot_creation


Notes accompanying the Robot Ignite Academy ROBOT CREATION WITH URDF Course https://www.robotigniteacademy.com/en/course/robot-creation-with-urdf-ros/details/


* In this unit, you will learn how to go from a physical robot to a visual virtual model. By visual, we understand that its not a physically working simulation model. It's only the barebones of what, at the end, will be used for simulation.  
* But this URDF Visual model is already very useful.  
* If you have a real robot and you want to use the ROS infrastructure, you need a virtual description of how the robot is connected and where each of the sensors is in some applications. For example, if you have a camera mounted on the head of the robot, through the virtual robot description (the URDF file ), you can use the TF ROS structure to know exactly where the camera is based by only the joint sensor readings.  
It also allows you to represent the robot model inside RVIZ.





# Create your first URDF Model[](https://i-08d56a1597871c9af.robotigniteacademy.com/jupyter/notebooks/Course_urdfROS_Unit_1.ipynb#Create-your-first-URDF-Model)

So, you will start by creating the URDF of the robot Mira.  

-   Learn how to use the URDF creation tools and the step-by-step procedure for creating a robot model.
-   Learn about the morphology of the robot you want to work with.
-   Obtain the 3D models that you will need in the correct format.
-   Generate the link and joint structure.
-   Test the movement of the joints.

So, let's get started.

## 1. Learn how to use the URDF creation tools and the creation procedure[](https://i-08d56a1597871c9af.robotigniteacademy.com/jupyter/notebooks/Course_urdfROS_Unit_1.ipynb#1.-Learn-how-to-use-the-URDF-creation-tools-and-the-creation-procedure)

* Let's create the URDF file in the appropriate ROS structure and let's create a ROS package for each robot model that you create.  
* It's a common practice to always create a "my_robot_description" package where you store all of the files that describe the robot. You will find this robotname_description package everywhere in ROS packages that have robot models defined.
* So create a ROS package named "

Execute in WebShell #1

  
cd /home/user/catkin_ws/src  
catkin_create_pkg my_mira_description rospy rviz controller_manager gazebo_ros joint_state_publisher robot_state_publisher  

Once you have the "my_mira_description" package, create the following folders inside it:  

-   launch
-   models
-   rviz_config
-   config
-   urdf
-   worlds

These folders are the ones that you will need to have a fully-functional simulated robot.  
Now, create a URDF file called "mira.urdf" in the "urdf" folder




















































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbMTE0NzI1MzQ5OSwtMTYxODE0NjYwNiwxNz
UyMTc5MTQzXX0=
-->