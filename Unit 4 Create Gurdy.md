# Unit 4 Gurdy

To practice and better learn how URDF are created and used, you are going to create, from scratch and with a little help, the Robot Model of Gurdy Robot.
![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/gurdy/gurdy_RDS2.png?raw=true)
![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/gurdy/gurdy_RDS.png?raw=true)

![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/gurdy/gurdy.gif?raw=true)

---
Follow the same procedure that we followed to create the Mira URDF. These are some points to guide you through the creation process:  

-   Discover how many links and joints will be needed to create this robot model. Decide which joints will be fixed, revolve, or be continuous.  
    This Link Tree will give you a good idea of the morphology of Gurdy's link structure. 
    
   * [rqt_tf_tree](http://wiki.ros.org/rqt_tf_tree)
```
rosrun rqt_tf_tree rqt_tf_tree
```
![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/gurdy/gurdy_link_tree.png?raw=true)

---
Generate the **urdf_to_graphiz** tree:
```
urdf_to_graphiz gurdy.urdf
```


![enter image description here](https://github.com/rwbot/urdf_robot_creation/blob/master/images/gurdy/gurdy_link_and_joints-1.png?raw=true)
---
Here you have a blueprint of the distances to place the most difficult part, which is the joints between the head and the legs. The units are millimeters:
![](https://github.com/rwbot/urdf_robot_creation/blob/master/images/gurdy/Medidas_Gurdy_head.PNG?raw=true)

---
You will also need the dimensions of the the basic geometry to have some idea of the way that everything is connected, and the collision elements geometry:  

 - base_link : box size="0.01 0.01 0.01"   head_link : cylinder
 - radius="0.05" length="0.04"   upper_leg_links : cylinder
 - length="0.06" radius="0.0025"   lower_leg_links : cylinder
 - length="0.06" radius="0.0015"   foot_links : sphere radius="0.008"

The **.dae** models in **`/home/user/catkin_ws/src/my_gurdy_description/models/gurdy/meshes`**
```
/models/gurdy/meshes/gurdy_head_v2.dae
/models/gurdy/meshes/gurdy_higherleg_v2.dae
/models/gurdy/meshes/gurdy_lowerleg_v2.dae 
```

-   Once you have the Visual URDF model, it's time to add the collisions, the actuators, and the sensors.  
    The collisions will be geometric figures, as in the Mira Example.  
    The actuators are now six instead of three, so make the appropriate changes.  
    The limits in effort and speed should be around: effort="1.0" and velocity="0.005," but it will depend on the joint.  
    The weights of the links are:  
    base_link : mass=None, there is no inertia needed because its a non-functional element.  
    head_link : mass value="0.01"  
    upper_leg_links : mass value="0.01"  
    lower_leg_links : mass value="0.01"  
    foot_links : mass value="0.01"  
    

-   Friction is vital in the foot links. Try different values and see the effect ont locomotion. These are some orientative values:  
```
kp = 1000.0  
kd = 1000.0  
mu1 = 10.0  
mu2 = 10.0  
```

-   PID are tricky. Use the rqt_reconfigure to get the correct values. These are some orientative values:  
    {p: 3.0, i: 1.0, d: 0.0}

---

# **gurdy.urdf**


```xml
<?xml version="1.0"?>
<robot name="gurdy">

<!--rosservice call /gazebo/deleteModel "model_name: 'mira'"-->
<!--roslaunch myMira_description spawnMira.launch-->

<!--You will also need the dimensions of the the basic geometry to have some idea of the way that everything is connected, and the collision elements geometry:-->
<!--For base_link there is no inertia needed because its a non-functional element-->

<!--base_link :         box size="0.01 0.01 0.01"                   mass=None, -->
<!--head_link :         cylinder radius="0.05"      length="0.04"   mass value="0.01"-->
<!--upperleg_links :   cylinder radius="0.0025"    length="0.06"   mass value="0.01"-->
<!--lowerleg_links :   cylinder radius="0.0015"    length="0.06"   mass value="0.01"-->
<!--foot_links :        sphere   radius="0.008"                     mass value="0.01"-->


<!--Friction is vital in the foot links. Try different values and see the effect on locomotion. These are some orientative values:-->
<!--kp = 1000.0-->
<!--kd = 1000.0-->
<!--mu1 = 10.0-->
<!--mu2 = 10.0-->

<!--PID are tricky. Use rqt_reconfigure to get correct values. These are some orientative values:-->
<!--{p: 3.0, i: 1.0, d: 0.0}-->
```

---
## Link Definitions
```xml

<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->


<link name="base_link">
  <!-- base_link DOESNT USE INERTIALS -->
    <!--<inertial>-->
    <!--    <origin/>-->
    <!--    <mass value="0.001"/>-->
    <!--    <inertia ixx="1.66666666667e-08" ixy="0.0" ixz="0.0" iyy="1.66666666667e-08" iyz="0.0" izz="1.66666666667e-08"/>-->
    <!--</inertial>-->
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <box size="0.01 0.01 0.01"/>
        </geometry>
    </collision>
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <box size="0.01 0.01 0.01"/>
        </geometry>
    </visual>
</link>

     <link name="head_link">
         <inertial>
             <origin xyz="0 0 0.02" rpy="0 0 0"/>
             <mass value="0.01"/>
             <inertia ixx="7.58333333333e-06" ixy="0.0" ixz="0.0" iyy="7.58333333333e-06" iyz="0.0" izz="1.25e-05"/>
         </inertial>
         <collision>
             <origin xyz="0 0 0.02" rpy="0 0 0"/>
             <geometry>
                 <cylinder radius="0.05" length="0.04"/>
                 <!-- <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_head_v2.dae"/> -->
             </geometry>
         </collision>
         <visual>
             <origin xyz="0 0 0" rpy="0 0 0"/>
             <geometry>
                 <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_head_v2.dae"/>
             </geometry>
         </visual>
     </link>

          <link name="upperlegM1_link">
              <inertial>
                  <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--UPPER-->
                  <mass value="0.01"/>
                  <!-- ORIGINAL value taken from Inertial_calculator.py -->
                  <!-- <inertia ixx="3.015625e-06" ixy="0.0" ixz="0.0" iyy="3.015625e-06" iyz="0.0" izz="3.125e-08"/> -->
                  <!-- SOLUTION INERTIA ++++ WAY LESS TWITCHING -->
                  <inertia ixx="3.015625e-05" ixy="0.0" ixz="0.0" iyy="3.015625e-05" iyz="0.0" izz="3.125e-07"/>
              </inertial>
              <collision>
                  <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--UPPER COLLISION-->
                  <geometry>
                      <cylinder radius="0.0025" length="0.06"/>
                  </geometry>
              </collision>
              <visual>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <geometry>
                      <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_higherleg_v2.dae"/>
                  </geometry>
              </visual>
          </link>

          <link name="upperlegM2_link">
              <inertial>
                  <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--UPPER-->
                  <mass value="0.01"/>
                  <!-- ORIGINAL value taken from Inertial_calculator.py -->
                  <!-- <inertia ixx="3.015625e-06" ixy="0.0" ixz="0.0" iyy="3.015625e-06" iyz="0.0" izz="3.125e-08"/> -->
                  <!-- SOLUTION INERTIA ++++ WAY LESS TWITCHING -->
                  <inertia ixx="3.015625e-05" ixy="0.0" ixz="0.0" iyy="3.015625e-05" iyz="0.0" izz="3.125e-07"/>
              </inertial>
              <collision>
                  <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--UPPER COLLISION-->
                  <geometry>
                      <cylinder radius="0.0025" length="0.06"/>
                  </geometry>
              </collision>
              <visual>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <geometry>
                      <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_higherleg_v2.dae"/>
                  </geometry>
              </visual>
          </link>

          <link name="upperlegM3_link">
              <inertial>
                  <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--UPPER-->
                  <mass value="0.01"/>
                  <!-- ORIGINAL value taken from Inertial_calculator.py -->
                  <!-- <inertia ixx="3.015625e-06" ixy="0.0" ixz="0.0" iyy="3.015625e-06" iyz="0.0" izz="3.125e-08"/> -->
                  <!-- SOLUTION INERTIA ++++ WAY LESS TWITCHING -->
                  <inertia ixx="3.015625e-05" ixy="0.0" ixz="0.0" iyy="3.015625e-05" iyz="0.0" izz="3.125e-07"/>
              </inertial>
              <collision>
                  <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--UPPER COLLISION-->
                  <geometry>
                      <cylinder radius="0.0025" length="0.06"/>
                  </geometry>
              </collision>
              <visual>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <geometry>
                      <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_higherleg_v2.dae"/>
                  </geometry>
              </visual>
          </link>

               <link name="lowerlegM1_link">
                   <inertial>
                       <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER-->
                       <mass value="0.01"/>
                       <!-- ORIGINAL value taken from Inertial_calculator.py -->
                       <!-- <inertia ixx="3.005625e-06" ixy="0.0" ixz="0.0" iyy="3.005625e-06" iyz="0.0" izz="1.125e-08"/> -->
                       <!-- SOLUTION INERTIA ++++  -->
                       <inertia ixx="3.005625e-05" ixy="0.0" ixz="0.0" iyy="3.005625e-05" iyz="0.0" izz="1.125e-07"/>
                   </inertial>
                   <collision>
                       <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER COLLISION-->
                       <geometry>
                           <cylinder radius="0.0015" length="0.06"/>
                       </geometry>
                   </collision>
                   <visual>
                       <origin xyz="0 0 0" rpy="0 0 0"/>
                       <geometry>
                           <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_lowerleg_v2.dae"/>
                       </geometry>
                   </visual>
               </link>

               <link name="lowerlegM2_link">
                   <inertial>
                       <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER-->
                       <mass value="0.01"/>
                       <!-- ORIGINAL value taken from Inertial_calculator.py -->
                       <!-- <inertia ixx="3.005625e-06" ixy="0.0" ixz="0.0" iyy="3.005625e-06" iyz="0.0" izz="1.125e-08"/> -->
                       <!-- SOLUTION INERTIA ++++  -->
                       <inertia ixx="3.005625e-05" ixy="0.0" ixz="0.0" iyy="3.005625e-05" iyz="0.0" izz="1.125e-07"/>
                   </inertial>
                   <collision>
                       <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER COLLISION-->
                       <geometry>
                           <cylinder radius="0.0015" length="0.06"/>
                       </geometry>
                   </collision>
                   <visual>
                       <origin xyz="0 0 0" rpy="0 0 0"/>
                       <geometry>
                           <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_lowerleg_v2.dae"/>
                       </geometry>
                   </visual>
               </link>

               <link name="lowerlegM3_link">
                   <inertial>
                       <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER-->
                       <mass value="0.01"/>
                       <!-- ORIGINAL value taken from Inertial_calculator.py -->
                       <!-- <inertia ixx="3.005625e-06" ixy="0.0" ixz="0.0" iyy="3.005625e-06" iyz="0.0" izz="1.125e-08"/> -->
                       <!-- SOLUTION INERTIA ++++  -->
                       <inertia ixx="3.005625e-05" ixy="0.0" ixz="0.0" iyy="3.005625e-05" iyz="0.0" izz="1.125e-07"/>
                   </inertial>
                   <collision>
                       <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER COLLISION-->
                       <geometry>
                           <cylinder radius="0.0015" length="0.06"/>
                       </geometry>
                   </collision>
                   <visual>
                       <origin xyz="0 0 0" rpy="0 0 0"/>
                       <geometry>
                           <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_lowerleg_v2.dae"/>
                       </geometry>
                   </visual>
               </link>

                    <link name="footM1_link">
                        <inertial>
                            <origin xyz="0 0 0.01" rpy="0 0 0"/> <!--FOOT-->
                            <mass value="0.01"/>
                            <!-- ORIGINAL value taken from Inertial_calculator.py -->
                            <!-- <inertia ixx="2.56e-07" ixy="0.0" ixz="0.0" iyy="2.56e-07" iyz="0.0" izz="2.56e-07"/> -->
                            <!-- SOLUTION INERTIA ++++  -->
                            <inertia ixx="1.28e-06" ixy="0.0" ixz="0.0" iyy="1.28e-06" iyz="0.0" izz="1.28e-06"/>
                        </inertial>
                        <collision>
                            <origin xyz="0 0 0.01" rpy="0 0 0"/>
                            <geometry>
                                <sphere radius="0.008"/>
                            </geometry>
                        </collision>
                        <visual>
                            <origin xyz="0 0 0" rpy="0 0 0"/>
                            <geometry>
                                <sphere radius="0.008"/>
                            </geometry>
                        </visual>
                    </link>

                    <link name="footM2_link">
                        <inertial>
                            <origin xyz="0 0 0.01" rpy="0 0 0"/> <!--FOOT-->
                            <mass value="0.01"/>
                            <!-- ORIGINAL value taken from Inertial_calculator.py -->
                            <!-- <inertia ixx="2.56e-07" ixy="0.0" ixz="0.0" iyy="2.56e-07" iyz="0.0" izz="2.56e-07"/> -->
                            <!-- SOLUTION INERTIA ++++  -->
                            <inertia ixx="1.28e-06" ixy="0.0" ixz="0.0" iyy="1.28e-06" iyz="0.0" izz="1.28e-06"/>
                        </inertial>
                        <collision>
                            <origin xyz="0 0 0.01" rpy="0 0 0"/>
                            <geometry>
                                <sphere radius="0.008"/>
                            </geometry>
                        </collision>
                        <visual>
                            <origin xyz="0 0 0" rpy="0 0 0"/>
                            <geometry>
                                <sphere radius="0.008"/>
                            </geometry>
                        </visual>
                    </link>

                    <link name="footM3_link">
                        <inertial>
                            <origin xyz="0 0 0.01" rpy="0 0 0"/> <!--FOOT-->
                            <mass value="0.01"/>
                            <!-- ORIGINAL value taken from Inertial_calculator.py -->
                            <!-- <inertia ixx="2.56e-07" ixy="0.0" ixz="0.0" iyy="2.56e-07" iyz="0.0" izz="2.56e-07"/> -->
                            <!-- SOLUTION INERTIA ++++  -->
                            <inertia ixx="1.28e-06" ixy="0.0" ixz="0.0" iyy="1.28e-06" iyz="0.0" izz="1.28e-06"/>
                        </inertial>
                        <collision>
                            <origin xyz="0 0 0.01" rpy="0 0 0"/>
                            <geometry>
                                <sphere radius="0.008"/>
                            </geometry>
                        </collision>
                        <visual>
                            <origin xyz="0 0 0" rpy="0 0 0"/>
                            <geometry>
                                <sphere radius="0.008"/>
                            </geometry>
                        </visual>
                    </link>
```

---
## Joint Definitions
```xml
<!-- ################### JOINT Definitions ############################# -->
<!-- ################### JOINT Definitions ############################# -->
<!-- ################### JOINT Definitions ############################# -->
<!-- ################### JOINT Definitions ############################# -->
<!-- ################### JOINT Definitions ############################# -->

<joint name="base_head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
    <!--Rest NOT NEEDED for FIXED joint-->
    <!--<limit lower="-0.2" upper="0.2" effort="1.0" velocity="0.005"/>-->
    <!--<axis xyz="0 0 0"/>-->
</joint>

    <joint name="head_upperlegM1_joint" type="revolute">
        <parent link="head_link"/>
        <child link="upperlegM1_link"/>
        <!--this is the Pitch value from the urdf to graphiz map pdf-->
        <!--<origin xyz="-0.02165 -0.0125 -0.008" rpy="3.14159 -1.05879e-22 0.523599"/>-->
        <!--This Pitch value is from solution-->
        <!--Haven't seen a difference-->
        <origin xyz="-0.02165 -0.0125 -0.008" rpy="3.14159 0 0.523599"/>
        <!--Rest NOT NEEDED for FIXED joint-->
        <limit lower="-1.57" upper="0.0" effort="1.0" velocity="0.005"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="head_upperlegM2_joint" type="revolute">
        <parent link="head_link"/>
        <child link="upperlegM2_link"/>
        <origin xyz="0.02165 -0.0125 -0.008" rpy="3.14159 0 2.61799"/>
        <!--Rest NOT NEEDED for FIXED joint-->
        <limit lower="-1.57" upper="0.0" effort="1.0" velocity="0.005"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="head_upperlegM3_joint" type="revolute">
        <parent link="head_link"/>
        <child link="upperlegM3_link"/>
        <origin xyz="0 0.025 -0.008" rpy="3.14159 0 -1.5708"/>
        <!--Rest NOT NEEDED for FIXED joint-->
        <limit lower="-1.57" upper="0.0" effort="1.0" velocity="0.005"/>
        <axis xyz="0 1 0"/>
    </joint>

        <joint name="upperlegM1_lowerlegM1_joint" type="revolute">
            <parent link="upperlegM1_link"/>
            <child link="lowerlegM1_link"/>
            <origin xyz="0 0.0095 0.06" rpy="0 0 3.14159"/>
            <limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>
            <axis xyz="0 1 0"/>
        </joint>

        <joint name="upperlegM2_lowerlegM2_joint" type="revolute">
            <parent link="upperlegM2_link"/>
            <child link="lowerlegM2_link"/>
            <origin xyz="0 0.0095 0.06" rpy="0 0 3.14159"/>
            <limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>
            <axis xyz="0 1 0"/>
        </joint>

        <joint name="upperlegM3_lowerlegM3_joint" type="revolute">
            <parent link="upperlegM3_link"/>
            <child link="lowerlegM3_link"/>
            <origin xyz="0 0.0095 0.06" rpy="0 0 3.14159"/>
            <limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>
            <axis xyz="0 1 0"/>
        </joint>

            <joint name="lowerlegM1_footM1_joint" type="fixed">
                <parent link="lowerlegM1_link"/>
                <child link="footM1_link"/>
                <origin xyz="0 0 0.06" rpy="0 0 0"/>
                <!--Rest NOT NEEDED for FIXED joint-->
                <!--<limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>-->
                <!--<axis xyz="0 1 0"/>-->
            </joint>

            <joint name="lowerlegM2_footM2_joint" type="fixed">
                <parent link="lowerlegM2_link"/>
                <child link="footM2_link"/>
                <origin xyz="0 0 0.06" rpy="0 0 0"/>
                <!--Rest NOT NEEDED for FIXED joint-->
                <!--<limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>-->
                <!--<axis xyz="0 1 0"/>-->
            </joint>

            <joint name="lowerlegM3_footM3_joint" type="fixed">
                <parent link="lowerlegM3_link"/>
                <child link="footM3_link"/>
                <origin xyz="0 0 0.06" rpy="0 0 0"/>
                <!--Rest NOT NEEDED for FIXED joint-->
                <!--<limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>-->
                <!--<axis xyz="0 1 0"/>-->
            </joint>
```

---
## Transmission Definitions
```xml

<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->

<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/gurdy</robotNamespace>
    </plugin>
</gazebo>

<transmission name="transmission_head_upperlegM1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="head_upperlegM1_joint">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_head_upperlegM1">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>

<transmission name="transmission_head_upperlegM2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="head_upperlegM2_joint">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_head_upperlegM2">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>

<transmission name="transmission_head_upperlegM3">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="head_upperlegM3_joint">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_head_upperlegM3">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>

        <transmission name="transmission_upperlegM1_lowerlegM1">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="upperlegM1_lowerlegM1_joint">
                <hardwareInterface>EffortJointInterface</hardwareInterface>
            </joint>
            <actuator name="motor_upperlegM1_lowerlegM1">
                <hardwareInterface>EffortJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>

        <transmission name="transmission_upperlegM2_lowerlegM2">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="upperlegM2_lowerlegM2_joint">
                <hardwareInterface>EffortJointInterface</hardwareInterface>
            </joint>
            <actuator name="motor_upperlegM2_lowerlegM2">
                <hardwareInterface>EffortJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>

        <transmission name="transmission_upperlegM3_lowerlegM3">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="upperlegM3_lowerlegM3_joint">
                <hardwareInterface>EffortJointInterface</hardwareInterface>
            </joint>
            <actuator name="motor_upperlegM3_lowerlegM3">
                <hardwareInterface>EffortJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>                     
			</transmission>
```

---

## Materials Definitions
```xml
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->

<gazebo reference="head_link">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
</gazebo>
      <!-- This is for color and physical properties in Gazebo, color won't work with the material tag in gazebo
      only for URDF coloring -->
      <gazebo reference="footM1_link">
          <kp>1000.0</kp>
          <kd>1000.0</kd>
          <mu1>10.0</mu1>
          <mu2>10.0</mu2>
          <!--<material>Gazebo/Red</material>-->
          <material>Gazebo/White</material>
      </gazebo>

      <!-- This is for color and physical properties in Gazebo, color won't work with the material tag in gazebo
      only for URDF coloring -->
      <gazebo reference="footM2_link">
          <kp>1000.0</kp>
          <kd>1000.0</kd>
          <mu1>10.0</mu1>
          <mu2>10.0</mu2>
          <!--<material>Gazebo/Green</material>-->
          <material>Gazebo/White</material>
      </gazebo>

      <!-- This is for color and physical properties in Gazebo, color won't work with the material tag in gazebo
      only for URDF coloring -->
      <gazebo reference="footM3_link">
          <kp>1000.0</kp>
          <kd>1000.0</kd>
          <mu1>10.0</mu1>
          <mu2>10.0</mu2>
          <!--<material>Gazebo/Blue</material>-->
          <material>Gazebo/White</material>
      </gazebo>


<!-- END -->
</robot>
```
---

## Sensor Definitions
-   As for sensors, you will have to add an IMU to the "base_link." This is the code for adding an IMU:
```xml
<!-- ################### SENSOR Definitions ############################# -->
<!-- ################### SENSOR Definitions ############################# -->
<!-- ################### SENSOR Definitions ############################# -->
<!-- ################### SENSOR Definitions ############################# -->
<!-- ################### SENSOR Definitions ############################# -->

<gazebo>
    <plugin name="gazebo_ros_imu_controller" filename="libgazebo_ros_imu.so">
        <robotNamespace>/gurdy</robotNamespace>
        <topicName>imu/data</topicName>
        <serviceName>imu/service</serviceName>
        <bodyName>base_link</bodyName>
        <gaussianNoise>0</gaussianNoise>
        <rpyOffsets>0 0 0</rpyOffsets>
        <updateRate>10.0</updateRate>
        <alwaysOn>true</alwaysOn>
        <gaussianNoise>0</gaussianNoise>
    </plugin>
</gazebo>
```

---

## Controllers

### gurdy_control.yaml
```yaml
gurdy:
    # Publish all joint states 
    # -----------------------------------
    joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50
    
    # Position Controllers 
    # Name the controller the exact same joint name, proceeded by "_position_controller"
    # ---------------------------------------
    head_upperlegM1_joint_position_controller:
        # Type: is JointPositionController because we want to control it through the position of the joints.
        type: effort_controllers/JointPositionController
        # The JointName: This is linking the joint to the controller.
        joint: head_upperlegM1_joint
        # Sets the PID values. This is absolutely vital to making the actuators reach the desired position without oscillations or going too slow.
        pid: {p: 3.0, i: 1.0, d: 0.0}
    head_upperlegM2_joint_position_controller:
        type: effort_controllers/JointPositionController
        joint: head_upperlegM2_joint
        pid: {p: 3.0, i: 1.0, d: 0.0}
    head_upperlegM3_joint_position_controller:
        type: effort_controllers/JointPositionController
        joint: head_upperlegM3_joint
        pid: {p: 3.0, i: 1.0, d: 0.0}
    upperlegM1_lowerlegM1_joint_position_controller:
        type: effort_controllers/JointPositionController
        joint: upperlegM1_lowerlegM1_joint
        pid: {p: 3.0, i: 1.0, d: 0.0}
    upperlegM2_lowerlegM2_joint_position_controller:
        type: effort_controllers/JointPositionController
        joint: upperlegM2_lowerlegM2_joint
        pid: {p: 3.0, i: 1.0, d: 0.0}
    upperlegM3_lowerlegM3_joint_position_controller:
        type: effort_controllers/JointPositionController
        joint: upperlegM3_lowerlegM3_joint
        pid: {p: 3.0, i: 1.0, d: 0.0}
```

---

## Launch Files

### spawn_gurdy.launch
```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--spawns the GURDY URDF file into the given point in space, -->
<!--if a gazebo simulation is running.-->
<launch>
   <include file="$(find my_gurdy_description)/launch/spawn_urdf.launch">
       <arg name="x" value="0.0"/>
       <arg name="y" value="0.0"/>
       <arg name="z" value="0.0"/>
       <!--<arg name="urdf_robot_file" value="$(find my_gurdy_description)/urdf/gurdy.urdf"/>-->
       <arg name="robot_name" value="gurdy"/>
   </include>
</launch>
```

### gurdy_control.launch
```xml
<launch>
<!--Loading the yaml file and launching the controllers 
with the robot state publisher for the TF publication-->

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find my_gurdy_description)/config/gurdy_control.yaml" command="load"/>

  <!-- load the controllers -->
  <!--Here you state that all controllers defined in the yaml are active.-->

  <node pkg="controller_manager" name="controller_spawner" 
  type="spawner" respawn="false"
    output="screen" ns="/gurdy" 
    args="head_upperlegM1_joint_position_controller 
        head_upperlegM2_joint_position_controller 
        head_upperlegM3_joint_position_controller       
            upperlegM1_lowerlegM1_joint_position_controller 
            upperlegM2_lowerlegM2_joint_position_controller 
            upperlegM3_lowerlegM3_joint_position_controller 
    joint_state_controller"/>

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node pkg="robot_state_publisher" name="robot_state_publisher" 
  type="robot_state_publisher" respawn="false" output="screen">
    <remap from="/joint_states" to="/gurdy/joint_states" />
  </node>

</launch>
```

### spawn_with_controllers.launch
```xml
<launch>
    <include file="$(find my_gurdy_description)/launch/spawn_gurdy.launch"/>
    <include file="$(find my_gurdy_description)/launch/gurdy_control.launch"/>
</launch>
```

---

## Make it Move

### gurdy_joint_mover.py

```python
#!/usr/bin/env python

import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/gurdy/head_upperlegM1_joint_position_controller/command
/gurdy/head_upperlegM2_joint_position_controller/command
/gurdy/head_upperlegM3_joint_position_controller/command
/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command
/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command
/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command
"""

class gurdyJointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("Gurdy JointMover Initialising...")

        self.pub_upperlegM1_joint_position = rospy.Publisher(
            '/gurdy/head_upperlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_upperlegM2_joint_position = rospy.Publisher(
            '/gurdy/head_upperlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_upperlegM3_joint_position = rospy.Publisher(
            '/gurdy/head_upperlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_lowerlegM1_joint_position = rospy.Publisher(
            '/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_lowerlegM2_joint_position = rospy.Publisher(
            '/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_lowerlegM3_joint_position = rospy.Publisher(
            '/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        joint_states_topic_name = "/gurdy/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.gurdy_joints_callback)
        gurdy_joints_data = None
        rate = rospy.Rate(2)
        while gurdy_joints_data is None:
            try:
                gurdy_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass
            rate.sleep()

        self.gurdy_joint_dictionary = dict(zip(gurdy_joints_data.name, gurdy_joints_data.position))

    def move_gurdy_all_joints(self, upperlegM1_angle, upperlegM2_angle, upperlegM3_angle, lowerlegM1_value, lowerlegM2_value ,lowerlegM3_value):
        upperlegM1 = Float64()
        upperlegM1.data = upperlegM1_angle
        upperlegM2 = Float64()
        upperlegM2.data = upperlegM2_angle
        upperlegM3 = Float64()
        upperlegM3.data = upperlegM3_angle

        lowerlegM1 = Float64()
        lowerlegM1.data = lowerlegM1_value
        lowerlegM2 = Float64()
        lowerlegM2.data = lowerlegM2_value
        lowerlegM3 = Float64()
        lowerlegM3.data = lowerlegM3_value

        self.pub_upperlegM1_joint_position.publish(upperlegM1)
        self.pub_upperlegM2_joint_position.publish(upperlegM2)
        self.pub_upperlegM3_joint_position.publish(upperlegM3)

        self.pub_lowerlegM1_joint_position.publish(lowerlegM1)
        self.pub_lowerlegM2_joint_position.publish(lowerlegM2)
        self.pub_lowerlegM3_joint_position.publish(lowerlegM3)


    def gurdy_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.gurdy_joint_dictionary = dict(zip(msg.name, msg.position))


    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def gurdy_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'base_waist_joint', 'body_head_joint', 'waist_body_joint is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.gurdy_joint_dictionary.get(joint_name)
        if not joint_reading:
            print "self.gurdy_joint_dictionary="+str(self.gurdy_joint_dictionary)
            print "joint_name===>"+str(joint_name)
            assert "There is no data about that joint"
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    def gurdy_movement_look(self, upperlegM1_angle, upperlegM2_angle, upperlegM3_angle, lowerlegM1_value, lowerlegM2_value ,lowerlegM3_value):
        """
        Move:
        'head_upperlegM1_joint',
        'head_upperlegM2_joint',
        'head_upperlegM3_joint',
        'upperlegM1_lowerlegM1_joint',
        'upperlegM2_lowerlegM2_joint',
        'upperlegM3_lowerlegM3_joint'
        :return:
        """
        check_rate = 5.0
        position_upperlegM1 = upperlegM1_angle
        position_upperlegM2 = upperlegM2_angle
        position_upperlegM3 = upperlegM3_angle

        position_lowerlegM1 = lowerlegM1_value
        position_lowerlegM2 = lowerlegM2_value
        position_lowerlegM3 = lowerlegM3_value

        similar_upperlegM1 = False
        similar_upperlegM2 = False
        similar_upperlegM3 = False

        similar_lowerlegM1 = False
        similar_lowerlegM2 = False
        similar_lowerlegM3 = False

        rate = rospy.Rate(check_rate)
        while not (similar_upperlegM1 and similar_upperlegM2 and similar_upperlegM3 and similar_lowerlegM1 and similar_lowerlegM2 and similar_lowerlegM3):
            self.move_gurdy_all_joints(position_upperlegM1,
                                       position_upperlegM2,
                                       position_upperlegM3,
                                       position_lowerlegM1,
                                       position_lowerlegM2,
                                       position_lowerlegM3)
            similar_upperlegM1 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM1_joint",
                                                                         value=position_upperlegM1)
            similar_upperlegM2 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM2_joint",
                                                                         value=position_upperlegM2)
            similar_upperlegM3 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM3_joint",
                                                                         value=position_upperlegM3)
            similar_lowerlegM1 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM1_lowerlegM1_joint",
                                                                         value=position_lowerlegM1)
            similar_lowerlegM2 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM2_lowerlegM2_joint",
                                                                         value=position_lowerlegM2)
            similar_lowerlegM3 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM3_lowerlegM3_joint",
                                                                         value=position_lowerlegM3)

            rate.sleep()

    def gurdy_init_pos_sequence(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = -1.55
        upperlegM2_angle = -1.55
        upperlegM3_angle = -1.55
        lowerlegM1_angle = 0.0
        lowerlegM2_angle = 0.0
        lowerlegM3_angle = 0.0
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

        lowerlegM1_angle = -1.55
        lowerlegM2_angle = -1.55
        lowerlegM3_angle = -1.55
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

    def gurdy_hop(self, num_hops=15):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """

        upper_delta = 1
        basic_angle = -1.55
        angle_change = random.uniform(0.2, 0.7)
        upperlegM_angle = basic_angle
        lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0

        #self.gurdy_init_pos_sequence()
        for repetitions in range(num_hops):
            self.gurdy_movement_look(upperlegM_angle,
                                     upperlegM_angle,
                                     upperlegM_angle,
                                     lowerlegM_angle,
                                     lowerlegM_angle,
                                     lowerlegM_angle)

            upper_delta *= -1
            if upper_delta < 0:
                upperlegM_angle = basic_angle + angle_change
            else:
                upperlegM_angle = basic_angle
            lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0


    def gurdy_moverandomly(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = random.uniform(-1.55, 0.0)
        upperlegM2_angle = random.uniform(-1.55, 0.0)
        upperlegM3_angle = random.uniform(-1.55, 0.0)
        lowerlegM1_angle = random.uniform(-2.9, pi/2)
        lowerlegM2_angle = random.uniform(-2.9, pi/2)
        lowerlegM3_angle = random.uniform(-2.9, pi/2)
        self.gurdy_movement_look(upperlegM1_angle,
                                 upperlegM2_angle,
                                 upperlegM3_angle,
                                 lowerlegM1_angle,
                                 lowerlegM2_angle,
                                 lowerlegM3_angle)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Gurdy...")
        while not rospy.is_shutdown():
            self.gurdy_init_pos_sequence()
            #self.gurdy_moverandomly()
            self.gurdy_hop()

if __name__ == "__main__":
    gurdy_jointmover_object = gurdyJointMover()
    gurdy_jointmover_object.movement_random_loop()
```
