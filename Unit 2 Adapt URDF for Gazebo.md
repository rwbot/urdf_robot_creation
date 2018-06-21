# # Unit 2: Adapt URDF for Gazebo Simulator

URDF files are very useful, as you have seen, for having a virtual representation of the different links and joints.  
But you are not simulating its weight, its inertia, what sensors it has, how it collides with other objects, the friction with the floor, or how the server position control will react to the robot.

## 1. Add Collisions
One of the most crucial elements in being able to simulate a robot is how it interacts with the world around it.  
At the moment, your URDF Mira Robot would be a Ghost in a simulation. There is a visual representation, but it can't interact with anything. It will go through the floor and the objects.  So, the first thing you have to do is add collisions to your URDF model. The only difference is that there is a new tag called **collision** that specifies the collision geometry. This is the shape used for calculating the physical contacts.
```xml
<link name="base_link">
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.06" length="0.09"/>
                <mesh filename="package://mira_description/models/mira/meshes/mira_body_v3.dae"/>
                <mesh filename="package://mira_description/models/mira/meshes/mira_body_v3_lowpolygons.dae"/>
            </geometry>
        </collision>
        <visual>
			...
        </visual>
</link>
```















































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTEyNDg5MDUxMjksMTAwMDI4NzI2XX0=
-->