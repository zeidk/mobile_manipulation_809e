#!/usr/bin/env python


# python
import sys
import copy
import os
import yaml
import re
from math import pi, sqrt
import yaml
import io
from pprint import pprint
# ros
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf.transformations import quaternion_multiply, quaternion_from_euler
# moveit
import moveit_commander as mc

class MoveitKairos():

    def __init__(self, node_name='moveit_kairos_809e', ns='',
                 robot_description='robot_description'):

        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=False)

        self.locations={}
        # Joint positions for the arm

        # robot_arm_shoulder_pan_joint
        # robot_arm_shoulder_lift_joint
        # robot_arm_elbow_joint
        # robot_arm_wrist_1_joint
        # robot_arm_wrist_2_joint
        # robot_arm_wrist_3_joint
       
        
        name = 'home'
        kairos_arm = [pi/4, 0, -2, 2, pi/2, -pi/2]
        self.locations[name] = (kairos_arm)

        name = 'check'
        kairos_arm = [-4.02, -0.13, -0.75, -0.63, 0, 0]
        self.locations[name] = (kairos_arm)

        name = 'retract'
        kairos_arm = [-4.02, -0.13, -2.45, 0.75, 0, 0]
        self.locations[name] = (kairos_arm)


        self.robot = mc.RobotCommander(ns + '/' + robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)
        # Define MoveIt groups
        self.kairos_moveit_groups = ['ur5_arm', 'egh_gripper']

        self.groups = {}
        for group_name in self.kairos_moveit_groups:
            group = mc.MoveGroupCommander(
                group_name,
                robot_description=ns + '/' + robot_description,
                ns=ns
            )
            group.set_goal_tolerance(0.05)
            self.groups[group_name] = group
        # self.goto_preset_location("check")
        # self.go_home()

    def go_home(self):
        self.goto_preset_location('home')


    def goto_preset_location(self, location_name):

        group = self.groups['ur5_arm']

        kairos_arm = self.locations[location_name]
        location_pose = group.get_current_joint_values()
 
        location_pose[:] = kairos_arm
        
        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 5
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            assert(attempts < MAX_ATTEMPTS)

    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")

    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        config_name = None
        if rospy.has_param('~config'):
            config_name = rospy.get_param("~config")
            if config_name == "check":
                self.goto_preset_location("retract")
                self.goto_preset_location("check")
            elif config_name == "home":
                self.goto_preset_location("retract")
                self.goto_preset_location("home")
            # elif config_name == "retract":
            #     self.goto_preset_location("home")
            else:
                rospy.logerr("Unknown arm configuration")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)