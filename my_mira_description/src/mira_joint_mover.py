#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/mira/pitch_joint_position_controller/command
/mira/roll_joint_position_controller/command
/mira/yaw_joint_position_controller/command
"""

class MiraJointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("Mira JointMover Initialising...")
        self.mira_roll_joint_position_publisher = rospy.Publisher('/mira/roll_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.mira_pitch_joint_position_publisher = rospy.Publisher('/mira/pitch_joint_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.mira_yaw_joint_position_publisher = rospy.Publisher('/mira/yaw_joint_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
        joint_states_topic_name = "/mira/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.mira_joints_callback)
        mira_joints_data = None
        while mira_joints_data is None:
            try:
                mira_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass

        self.mira_joint_dictionary = dict(zip(mira_joints_data.name, mira_joints_data.position))


    def move_mira_all_joints(self, roll, pitch, yaw):
        angle_roll = Float64()
        angle_roll.data = roll
        angle_pitch = Float64()
        angle_pitch.data = pitch
        angle_yaw = Float64()
        angle_yaw.data = yaw
        self.mira_roll_joint_position_publisher.publish(angle_roll)
        self.mira_pitch_joint_position_publisher.publish(angle_pitch)
        self.mira_yaw_joint_position_publisher.publish(angle_yaw)


    def move_mira_roll_joint(self, position):
        """
        limits radians : lower="-0.2" upper="0.2"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.mira_roll_joint_position_publisher.publish(angle)


    def move_mira_pitch_joint(self, position):
        """
        limits radians : lower="0" upper="0.44"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.mira_pitch_joint_position_publisher.publish(angle)


    def move_mira_yaw_joint(self, position):
        """
        Limits : continuous, no limits
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.mira_yaw_joint_position_publisher.publish(angle)


    def mira_joints_callback(self, msg):
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
        self.mira_joint_dictionary = dict(zip(msg.name, msg.position))


    def mira_check_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        :param value:
        :return:
        """
        similar = self.mira_joint_dictionary.get(joint_name) >= (value - error ) and self.mira_joint_dictionary.get(joint_name) <= (value + error )
        return similar


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


    def mira_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param value:
        :return:
        """
        joint_reading = self.mira_joint_dictionary.get(joint_name)
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error
        return similar


    def mira_movement_sayno(self):
        """
        Make Mira say no with the head
        :return:
        """
        check_rate = 5.0
        position = 0.7

        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar = False
            while not similar:
                self.move_mira_yaw_joint(position=position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar = self.mira_check_continuous_joint_value(joint_name="yaw_joint", value=position)

                rate.sleep()
            position *= -1



    def mira_movement_look(self, roll, pitch, yaw):
        """
        Make Mira look down
        :return:
        """
        check_rate = 5.0
        position_roll = roll
        position_pitch = pitch
        position_yaw = yaw

        similar_roll = False
        similar_pitch = False
        similar_yaw = False
        rate = rospy.Rate(check_rate)
        while not (similar_roll and similar_pitch and similar_yaw):
            self.move_mira_all_joints(position_roll, position_pitch, position_yaw)
            similar_roll = self.mira_check_continuous_joint_value(joint_name="roll_joint", value=position_roll)
            similar_pitch = self.mira_check_continuous_joint_value(joint_name="pitch_joint", value=position_pitch)
            similar_yaw = self.mira_check_continuous_joint_value(joint_name="yaw_joint", value=position_yaw)
            rate.sleep()


    def mira_lookup(self):
        self.mira_movement_look(roll=0.0,pitch=0.3,yaw=1.57)


    def mira_lookdown(self):
        self.mira_movement_look(roll=0.0,pitch=0.0,yaw=1.57)


    def mira_movement_laugh(self, set_rpy=False, roll=0.05, pitch=0.0, yaw=1.57, n_giggle=15):
        """
        Giggle in a given pitch yaw configuration
        :return:
        """
        position_roll = roll
        position_pitch = pitch
        position_yaw = yaw
        for repetitions in range(n_giggle):
            if set_rpy:
                self.move_mira_all_joints(position_roll, position_pitch, position_yaw)
            else:
                self.move_mira_roll_joint(position_roll)
            time.sleep(0.1)
            position_roll *= -1


    def mira_moverandomly(self):
        roll = random.uniform(-0.15, 0.15)
        pitch = random.uniform(0.0, 0.3)
        yaw = random.uniform(0.0, 2*pi)
        self.mira_movement_look(roll, pitch, yaw)


    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Mira...")
        while not rospy.is_shutdown():
            self.mira_moverandomly()
            self.mira_movement_laugh()


if __name__ == "__main__":
    mira_jointmover_object = MiraJointMover()
    mira_jointmover_object.movement_random_loop()

