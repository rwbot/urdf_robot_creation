# Unit 4: XACRO files

As you might have experienced already, very simple robots can have really big and complex URDF files.  
For this reason and to be able make URDF files more flexible for change, XACROS are used in Robot Model definitions even more than URDF files.

XACROS also give some tools for making simple Math operations inside the URDF files, as well as having Constants and MACROS.

## 1. Basics on using XACRO
Xacro is based on a macro language. All Xacro files end up being a single URDF file, but this way, you will have much cleaner and easier-to-maintain robot model definitions.

#### Generate a URDF from a XACRO file by:
```
rosrun xacro xacro model.xacro > model.urdf
```

But the best way is to generate the URDF files on the fly each time you execute a launch file. This way, you will occupy less memory. The only drawback is the fact that it has to be generated each time the launch is executed, which is slower than using a URDF directly.

```xml
<param name="robot_description"
  command="$(find xacro)/xacro '$(find my_robot_description)/robots/myrobot.urdf.xacro'" />
  ```

### It's common to put the XACRO files inside the "robots" folder.


## 2. Using XACRO

It's very important that you place the plugin for the control before the MACROS; otherwise, it won't be loaded, and could give you the warning: **Controller Spawner couldn't find the expected controller_manager ROS interface.** The first line in a XACRO file **must be**:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gurdy">
```

### Constants

One of the easiest ways to optimize a URDF is by creating constants to avoid having to change the same value in different places. So, let's look for values that are shared and could be simplified. And because Gurdy is made up of the same repeated elements, it's highly probable that a lot of elements can be made of shared constants. Some links share the same exact mass, so why don't we create a constant that stores the mass that is shared by all the links of the Gurdy Robot? There are also repeated values in each link because the origin of all its sub-elements is the same, so that's another candidate for XACRO simplification.

* Define a XACRO constant:
```xml
<xacro:property name="my_constant" value="my_constants_value" />
```
---
### Math
You can perform the basic operations (+,-,*,/), the unary minus, and use parenthesis. This allows you to, for example, calculate the inertias based on the dimensions directly in the XACRO file. Another example would be symmetrically-positioned links or joints.
* Calculate an inertial:
```xml
<xacro:property name="head_ixx" value="${( (m/12.0) * (3*head_radius*head_radius + head_length*head_length) )}"/>
```

---
# 3. MACROS

## Simple Macro
* Define a simple MACRO
```xml
<xacro:macro name="simple_macro_1">
    <block of code/>
</xacro:macro>
```
* Use a simple MACRO in another simple MACRO
```xml
<xacro:macro name="simple_macro_2">
    <xacro:simple_macro_1/>
</xacro:macro>
```
* Example of Simple MACROs:
```xml
<xacro:macro name="default_origin">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>

<xacro:macro name="visual_origin">
    <xacro:default_origin/>
</xacro:macro>
```

___

## Parametrised MACRO

Macros can have input parameters. This input can be values or even entire blocks:

* Defining a Macro with Value Parameter & Block Parameter, denoted with a **`*`**
```xml
<xacro:macro name="block_param_macro" params="value_param *block_param">
  <new_block name="example_${value_param}">
	<xacro:insert_block name="block_param"/>
  </new_block>
</xacro:macro>
```

* Calling a Parameterised MACRO:
```xml
<link name="upper_link">
  <xacro:name_of_macro? value_param="val">
	<block_param_input>
  </xacro:name_of_macro>
</link>
```

Example Defining:
```xml
<xacro:macro name="blue_shape" params="mass *shape">
    
   <inertial>
         <origin xyz="0 0 0.03" rpy="0 0 0"/>
         <mass value="${mass}" />
         <inertia ixx="3.015625e-05" ixy="0.0" ixz="0.0" iyy="3.015625e-05" iyz="0.0" izz="3.125e-07"/>
     </inertial>
     <collision>
         <geometry>         
             <xacro:insert_block name="shape" />             
         </geometry>
     </collision>
</xacro:macro>

```
* Calling Example
```xml
<link name="upperleg_M1_link">
    <xacro:upper_leg mass="0.01">
        <cylinder length="0.06" radius="0.0025"/>
    </xacro:blue_shape>
</link>
```

---

# gurdy.urdf.xacro

## XACROs and MACROs 
```xml
<?xml version="1.0"?>
<!-- <robot name="gurdy"> -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gurdy">

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

<!-- ################### XACRO Definitions ############################# -->
<!-- ################### XACRO Definitions ############################# -->
<!-- ################### XACRO Definitions ############################# -->
<!-- ################### XACRO Definitions ############################# -->
<!-- ################### XACRO Definitions ############################# -->

<!-- MUST ALWAYS GO BEFORE XACRO DEFINITIONS -->
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/gurdy</robotNamespace>
    </plugin>
</gazebo>

<!-- You can perform the basic operations (+,-,*,/), the unary minus, and use parenthesis. This allows you to, for example, calculate the inertias based on the dimensions directly in the XACRO file. -->
<xacro:property name="m" value="0.01"/>
<xacro:property name="head_radius" value="0.05"/>
<xacro:property name="head_length" value="0.04"/>
<xacro:property name="head_z_origin" value="0.02"/>

<xacro:property name="head_ixx" value="${( (m/12.0) * (3*head_radius*head_radius + head_length*head_length) )}"/>
<!-- Another example would be symmetrically-positioned links or joints. -->
<xacro:property name="head_iyy" value="${head_ixx}"/>
<xacro:property name="head_izz" value="${( (m*head_radius*head_radius) / (2.0) )}"/>



<!-- MACROS
This is by far the most useful feature of XACROS. There are two main types of MACROS: -->
<!-- SIMPLE MACRO -->
<xacro:macro name="default_origin">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>
<xacro:macro name="visual_origin">
    <xacro:default_origin/>
</xacro:macro>


<!--######################## UPPER LEG XACROS ########################-->
<!--######################## UPPER LEG XACROS ########################-->
<!--######################## UPPER LEG XACROS ########################-->
<!--######################## UPPER LEG XACROS ########################-->
<xacro:property name="upperleg_radius" value="0.0025"/>
<xacro:property name="upperleg_length" value="0.06"/>
<xacro:macro name="upperleg_origin">
    <origin xyz="0 0 0.02" rpy="0 0 0"/> <!--UPPER-->
</xacro:macro>
<xacro:macro name="upperleg_link_inertia">
    <!-- ORIGINAL value taken from Inertial_calculator.py -->
    <!-- <inertia ixx="3.015625e-06" ixy="0.0" ixz="0.0" iyy="3.015625e-06" iyz="0.0" izz="3.125e-08"/> -->
    <!-- SOLUTION INERTIA ++++ WAY LESS TWITCHING -->
    <inertia ixx="3.015625e-05" ixy="0.0" ixz="0.0" iyy="3.015625e-05" iyz="0.0" izz="3.125e-07"/>
</xacro:macro>


<!-- PARAMETERISED MACRO -->
<xacro:macro name="generate_upperleg_link" params="num">
    <link name="upperlegM${num}_link"><!--UPPER-->
        <inertial>
            <xacro:upperleg_origin/>
            <mass value="${m}"/>
            <xacro:upperleg_link_inertia/>
        </inertial>
        <collision>
            <xacro:upperleg_origin/>
            <geometry>
                <cylinder radius="${upperleg_radius}" length="${upperleg_length}"/>
            </geometry>
        </collision>
        <visual>
            <xacro:visual_origin/>
            <geometry>
                <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_higherleg_v2.dae"/>
            </geometry>
        </visual>
    </link>
</xacro:macro>



<!--######################## LOWER LEG XACROS ########################-->
<!--######################## LOWER LEG XACROS ########################-->
<!--######################## LOWER LEG XACROS ########################-->
<!--######################## LOWER LEG XACROS ########################-->
<xacro:property name="lowerleg_radius" value="0.0015"/>
<xacro:property name="lowerleg_length" value="0.06"/>
<xacro:macro name="lowerleg_origin">
    <origin xyz="0 0 0.03" rpy="0 0 0"/> <!--LOWER-->
</xacro:macro>
<xacro:macro name="lowerleg_link_inertia">
    <!-- ORIGINAL value taken from Inertial_calculator.py -->
    <!-- <inertia ixx="3.005625e-06" ixy="0.0" ixz="0.0" iyy="3.005625e-06" iyz="0.0" izz="1.125e-08"/> -->
    <!-- SOLUTION INERTIA ++++  -->
    <inertia ixx="3.005625e-05" ixy="0.0" ixz="0.0" iyy="3.005625e-05" iyz="0.0" izz="1.125e-07"/>
</xacro:macro>

<!-- PARAMETERISED MACRO -->
<xacro:macro name="generate_lowerleg_link" params="num">
    <link name="lowerlegM${num}_link"><!--UPPER-->
        <inertial>
            <xacro:lowerleg_origin/>
            <mass value="${m}"/>
            <xacro:lowerleg_link_inertia/>
        </inertial>
        <collision>
            <xacro:lowerleg_origin/>
            <geometry>
                <cylinder radius="${lowerleg_radius}" length="${lowerleg_length}"/>
            </geometry>
        </collision>
        <visual>
            <xacro:visual_origin/>
            <geometry>
                <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_lowerleg_v2.dae"/>
            </geometry>
        </visual>
    </link>
</xacro:macro>


<!--######################## FOOT XACROS ########################-->
<!--######################## FOOT XACROS ########################-->
<!--######################## FOOT XACROS ########################-->
<!--######################## FOOT XACROS ########################-->
<xacro:property name="foot_radius" value="0.008"/>
<xacro:macro name="foot_origin">
    <origin xyz="0 0 0.01" rpy="0 0 0"/> <!--FOOT-->
</xacro:macro>
<xacro:macro name="foot_link_inertia">
    <!-- ORIGINAL value taken from Inertial_calculator.py -->
    <!-- <inertia ixx="2.56e-07" ixy="0.0" ixz="0.0" iyy="2.56e-07" iyz="0.0" izz="2.56e-07"/> -->
    <!-- SOLUTION INERTIA ++++  -->
    <inertia ixx="1.28e-06" ixy="0.0" ixz="0.0" iyy="1.28e-06" iyz="0.0" izz="1.28e-06"/>
</xacro:macro>

<!-- PARAMETERISED MACRO -->
<xacro:macro name="generate_foot_link" params="num">
    <link name="footM${num}_link"><!--UPPER-->
        <inertial>
            <xacro:foot_origin/>
            <mass value="${m}"/>
            <xacro:foot_link_inertia/>
        </inertial>
        <collision>
            <xacro:foot_origin/>
            <geometry>
                <sphere radius="${foot_radius}"/>
            </geometry>
        </collision>
        <visual>
            <xacro:visual_origin/>
            <geometry>
                <sphere radius="${foot_radius}"/>
            </geometry>
        </visual>
    </link>
</xacro:macro>

<!--######################## JOINT XACROS ########################-->
<!--######################## JOINT XACROS ########################-->
<!--######################## JOINT XACROS ########################-->
<!--######################## JOINT XACROS ########################-->


<xacro:macro name="upperleg_lowerleg_joint_origin">
    <origin xyz="0 0.0095 0.06" rpy="0 0 3.14159"/>
</xacro:macro>
<xacro:macro name="upperleg_lowerleg_joint_limit">
    <limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>
</xacro:macro>
<xacro:macro name="generate_upperleg_lowerleg_joint" params="num">
    <joint name="upperlegM${num}_lowerlegM${num}_joint" type="revolute">
        <parent link="upperlegM${num}_link"/>
        <child link="lowerlegM${num}_link"/>
        <xacro:upperleg_lowerleg_joint_origin/>
        <xacro:upperleg_lowerleg_joint_limit/>
        <axis xyz="0 1 0"/>
    </joint>
</xacro:macro>


<xacro:macro name="lowerleg_foot_joint_origin">
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
</xacro:macro>
<xacro:macro name="generate_lowerleg_foot_joint" params="num">
    <joint name="lowerlegM${num}_footM${num}_joint" type="fixed">
        <parent link="lowerlegM${num}_link"/>
        <child link="footM${num}_link"/>
        <xacro:lowerleg_foot_joint_origin/>
        <!--Rest NOT NEEDED for FIXED joint-->
        <!--<limit lower="-2.65" upper="0" effort="1.0" velocity="0.005"/>-->
        <!--<axis xyz="0 1 0"/>-->
    </joint>
</xacro:macro>

<!--######################## TRANSMISSION XACROS ########################-->
<!--######################## TRANSMISSION XACROS ########################-->
<!--######################## TRANSMISSION XACROS ########################-->
<!--######################## TRANSMISSION XACROS ########################-->

<xacro:macro name="generate_head_upperleg_transmission" params="num" >
    <transmission name="transmission_head_upperlegM${num}">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="head_upperlegM${num}_joint">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor_head_upperlegM${num}">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
</xacro:macro>

<xacro:macro name="generate_upperleg_lowerleg_transmission" params="num" >
    <transmission name="transmission_upperlegM${num}_lowerlegM${num}">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="upperlegM${num}_lowerlegM${num}_joint">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor_upperlegM${num}_lowerlegM${num}">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
</xacro:macro>


<!--######################## GAZEBO REFERENCE XACROS ########################-->
<!--######################## GAZEBO REFERENCE XACROS ########################-->
<!--######################## GAZEBO REFERENCE XACROS ########################-->
<!--######################## GAZEBO REFERENCE XACROS ########################-->

<xacro:macro name="generate_foot_gazebo_reference" params="num color=White">
    <!-- <material>color</material> only works for rviz URDF, not Gazebo -->
    <!-- If 'color' param is unspecified, it defaults to White -->
        <gazebo reference="footM${num}_link">
        <kp>1000.0</kp>
        <kd>1000.0</kd>
        <mu1>10.0</mu1>
        <mu2>10.0</mu2>
        <xacro:if value="${color == ''}">
            <material>Gazebo/White</material>
        </xacro:if>
        <xacro:if value="${color != ''}">
            <material>Gazebo/${color}</material>
        </xacro:if>
        
        </gazebo>
</xacro:macro>
```

---

## Body Definitions

```xml

<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->
<!-- ################### LINK Definitions ############################# -->


    <link name="base_link">
        <!--<inertial>-->
        <!--    <origin/>-->
        <!--    <mass value="0.001"/>-->
        <!--    <inertia ixx="1.66666666667e-08" ixy="0.0" ixz="0.0" iyy="1.66666666667e-08" iyz="0.0" izz="1.66666666667e-08"/>-->
        <!--</inertial>-->
        <collision>
            <xacro:default_origin/>
            <geometry>
                <box size="0.01 0.01 0.01"/>
            </geometry>
        </collision>
        <visual>
            <xacro:visual_origin/>
            <geometry>
                <box size="0.01 0.01 0.01"/>
            </geometry>
        </visual>
    </link>

            <link name="head_link">
                <inertial>
                    <origin xyz="0 0 ${head_z_origin}" rpy="0 0 0"/>
                    <mass value="${m}"/>
                    <!-- <inertia ixx="7.58333333333e-06" ixy="0.0" ixz="0.0" iyy="7.58333333333e-06" iyz="0.0" izz="1.25e-05"/> -->
                    <inertia ixx="${head_ixx}" ixy="0.0" ixz="0.0" iyy="${head_iyy}" iyz="0.0" izz="${head_izz}"/>
                </inertial>
                <collision>
                    <origin xyz="0 0 ${head_z_origin}" rpy="0 0 0"/>
                    <geometry>
                        <cylinder radius="${head_radius}" length="${head_length}"/>
                        <!-- <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_head_v2.dae"/> -->
                    </geometry>
                </collision>
                <visual>
                    <xacro:visual_origin/>
                    <geometry>
                        <mesh filename="package://gurdy_description/models/gurdy/meshes/gurdy_head_v2.dae"/>
                    </geometry>
                </visual>
            </link>
                    <xacro:generate_upperleg_link num="1" />
                    <xacro:generate_upperleg_link num="2" />
                    <xacro:generate_upperleg_link num="3" />

                            <xacro:generate_lowerleg_link num="1" />
                            <xacro:generate_lowerleg_link num="2" />
                            <xacro:generate_lowerleg_link num="3" />

                                    <xacro:generate_foot_link num="1" />
                                    <xacro:generate_foot_link num="2" />
                                    <xacro:generate_foot_link num="3" />


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

                    <xacro:generate_upperleg_lowerleg_joint num="1"/>
                    <xacro:generate_upperleg_lowerleg_joint num="2"/>
                    <xacro:generate_upperleg_lowerleg_joint num="3"/>

                            <xacro:generate_lowerleg_foot_joint num="1"/>
                            <xacro:generate_lowerleg_foot_joint num="2"/>
                            <xacro:generate_lowerleg_foot_joint num="3"/>



<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->
<!-- ################### TRANSMISSION Definitions ############################# -->

            <xacro:generate_head_upperleg_transmission num="1"/>
            <xacro:generate_head_upperleg_transmission num="2"/>
            <xacro:generate_head_upperleg_transmission num="3"/>

                <xacro:generate_upperleg_lowerleg_transmission num="1"/>
                <xacro:generate_upperleg_lowerleg_transmission num="2"/>
                <xacro:generate_upperleg_lowerleg_transmission num="3"/>



<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->
<!-- ################### MATERIALS Definitions ############################# -->

        <gazebo reference="head_link">
            <mu1>10.0</mu1>
            <mu2>10.0</mu2>
        </gazebo>

        <xacro:generate_foot_gazebo_reference num="1" color=""/>
        <xacro:generate_foot_gazebo_reference num="2" />
        <xacro:generate_foot_gazebo_reference num="3" color="White"/>
        <xacro:generate_foot_gazebo_reference num="3" color="Blue"/>



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


</robot>
```

---

## Launch Files

### spawn_gurdy_xacro.launch
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <arg name="x" default="0.0" />
    <arg name="y" default="0.0" />
    <arg name="z" default="0.0" />
    
    <!--<arg name="xacro_robot_file" default="" />-->
    <arg name="robot_name" default="" />

    <!-- Load the URDF into the ROS Parameter Server -->
    <!-- Converts XACRO to URDF at launch time and passes it -->
    <!-- Manual conversion     rosrun xacro xacro model.xacro > model.urdf    -->
    <param name="robot_description" command="$(find xacro)/xacro '$(find my_gurdy_description)/robots/gurdy.urdf.xacro'" />
    
    <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -x $(arg x) -y $(arg y) -z $(arg z)  -model $(arg robot_name) -param robot_description"/>
</launch>
```

### spawn_xacro_with_controllers.launch
```xml
<launch>
    <include file="$(find my_gurdy_description)/launch/spawn_gurdy_xacro.launch"/>
    <include file="$(find my_gurdy_description)/launch/gurdy_control.launch"/>
</launch>
```

---

As you may have seen, symmetry and repetition are the best features for XACROS to make a difference and simplify quite a bit.  
In essence, you can make macros for **one full leg,** and then you only have to use it with a different number three times. That's why all the macros have this **number** variable.  
In this example, you have the macros: **upperleg_link**, **lowerleg_link**, **foot_link**, **upper_transmission**, and the lower_transmission. It is very intuitive, and it has been relatively easy because of the naming of each link and joint.  
Try to use numbers if elements repeat in a URDF, and then afterwards, MACROS will be easier to apply.  
The only thing that could be made macro, but does not need obvious math, is the **head_upperlegMX_joint**, which has been left as is.

