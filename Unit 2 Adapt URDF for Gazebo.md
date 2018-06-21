# # Unit 2: Adapt URDF for Gazebo Simulator

URDF files are very useful, as you have seen, for having a virtual representation of the different links and joints.  
But you are not simulating its weight, its inertia, what sensors it has, how it collides with other objects, the friction with the floor, or how the server position control will react to the robot.

## 1. Add Collisions
One of the most crucial elements in being able to simulate a robot is how it interacts with the world around it.  
At the moment, your URDF Mira Robot would be a Ghost in a simulation. There is a visual representation, but it can't interact with anything. It will go through the floor and the objects.  
So, the first thing you have to do is add collisions to your URDF model.
```xml
<link name="base_link">
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.06" length="0.09"/>
            </geometry>
        </collision>
        <visual>
            <origin rpy="0.0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="package://mira_description/models/mira/meshes/mira_body_v3.dae"/>
            </geometry>
            <material name="grey"/>
        </visual>
</link>
```















































#
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE1MDk5MTc4LDEwMDAyODcyNl19
-->