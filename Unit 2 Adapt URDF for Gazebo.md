# # Unit 2: Adapt URDF for Gazebo Simulator

URDF files are very useful, as you have seen, for having a virtual representation of the different links and joints.  
But you are not simulating its weight, its inertia, what sensors it has, how it collides with other objects, the friction with the floor, or how the server position control will react to the robot.

## 1. Add Collisions
One of the most crucial elements in being able to simulate a robot is how it interacts with the world around it.  
At the moment, your URDF Mira Robot would be a Ghost in a simulation. There is a visual representation, but it can't interact with anything. It will go through the floor and the objects.  So, the first thing you have to do is add collisions to your URDF model. The only difference is that there is a new tag called **collision** that specifies the collision geometry. This is the shape used for calculating the physical contacts.
```xml
<link>
	 <collision>
	     <origin xyz="0 0 0" rpy="0 0 0"/>
	     <geometry>
	         <cylinder radius="0.06" length="0.09"/>
	         <!--We can also use meshes-->
	         <mesh filename="package://mira_description/models/mira/meshes/mira_body_v3.dae"/>
	         <!--But the physics calculations will be more intenseive, so if using a mesh, use a low-poly version-->
	         <mesh filename="package://mira_description/models/mira/meshes/mira_body_v3_lowpolygons.dae"/>
	     </geometry>
	 </collision>
</link>
```
We can use meshes just like for the visual tag. But this is not advised, as the physics calculations are more intensive as the mesh gets more complex.  That's why the collisions are normally basic geometric shapes, while the visuals are meshes.  Another alternative if the geometry of the contact is crucial, is to use a lower poly version of the virtual mesh. That way, the shape is maintained, but less calculation power is needed.

#### We will only be defining collisions for links `base_link` and `head_link` because the rest don't need them. The eyes and camera don't because they are too small or don't protrude to have any significant effect on collisions. As for the roll, pitch, and yaw, they don't have to have collisions because they are virtual links; they don't exist in reality and it would just cause problems that the real robot doesn't have.

## 2. Spawn a robot in Gazebo through URDF Files
To spawn a URDF defined robot in the simulated world we need two launch files:

**`spawn_urdf.launch`**:  This spawns the given URDF file into the given point in space if a gazebo simulation is running. You can call this first launch through a second launch, passing the necessary arguments to it:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <arg name="x" default="0.0"/>
    <arg name="y" default="0.0"/>
    <arg name="z" default="0.0"/>
    
    <arg name="urdf_robot_file" default=""/>
    <arg name="robot_name" default=""/>
    
     <!--This version was created because of some errors seen in the V1 that crashed Gazebo or went too slow in spawn -->
     <!--Load the URDF into the ROS Parameter Server -->
    <param name="robot_description" command="cat $(arg urdf_robot_file)"/>
    
     <!--Run a python script to send a service call to gazebo_ros to spawn a URDF robot -->
    <node pkg="gazebo_ros" name="urdf_spawner" type="spawn_model" respawn="false" output="screen" args="-urdf -x $(arg x) -y $(arg y) -z $(arg z) -model $(arg robot_name) -param robot_description"/>
    
</launch>
```

**`spawn_mira.launch`**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
   <include file="$(find my_mira_description)/launch/spawn_urdf.launch">-->
       <arg name="x" value="0.0"/>
       <arg name="y" value="0.0"/>
       <arg name="z" value="0.2"/>
       <arg name="urdf_robot_file" value="$(find my_mira_description)/urdf/mira.urdf"/>
       <arg name="robot_name" value="mira"/>
   </include>
</launch>
```








































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE3ODgxMjk3NDcsMTAwMDI4NzI2XX0=
-->