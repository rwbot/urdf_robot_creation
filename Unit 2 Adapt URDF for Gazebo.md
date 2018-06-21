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














































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTUwODY5NDcyOSwxMDAwMjg3MjZdfQ==
-->