Handy URDF ROS commands:

Unit 1:
Link Tree will give you a good idea of the morphology of Gurdy's link structure. 
[rqt_tf_tree](http://wiki.ros.org/rqt_tf_tree)
```
rosrun rqt_tf_tree rqt_tf_tree
```

You can also see the Link-Joint structure of any URDF file through the urdf_to_graphiz tool:
```
urdf_to_graphiz mira.urdf
```

Visualize URDF:
urdf_visualize.launch



Unit 2:

Spawn URDF
spawn_urdf.launch
---
To view all the models present in the simulation:
rosservice call /gazebo/get_world_properties "{}"
---
PID Settings
**Diagnosing PID Settings**: [https://www.flitetest.com/articles/p-i-and-sometimes-d-gains-in-a-nutshell](https://www.flitetest.com/articles/p-i-and-sometimes-d-gains-in-a-nutshell)

Also, you will set a sensible PID at the start, so you can then calibrate it during the execution of movements through: [**DynamicReconfigure**](http://wiki.ros.org/dynamic_reconfigure) and its GUI,  [**rqt_reconfigure**](http://wiki.ros.org/rqt_reconfigure).  
Through **rqt_reconfigure**, you can change any parameter that is made accessible at runtime. This way, you just have to move the robot and change the values until you see that it behaves in the correct way. Launch it like:

rosrun rqt_reconfigure rqt_reconfigure
---

To test that the controllers are working
1. Publish directly in the controllers topics:
```
rostopic pub /mira/roll_joint_position_controller/command std_msgs/Float64 "data: 0.2"
```

2. Publish through the **rqt_gui** Topic Publishing.
```
rosrun rqt_gui rqt_gui
```
![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/rqt_gui_plugin_topic_publisher.png?raw=true)
Introducing a sinus based on the time (i) and divided by the frequency, Mira should move its head side to side. 
![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/rqt_gui_pub_topic.png?raw=true)


Unit 3:


------

Unit 4 XACRO
Gnerate
