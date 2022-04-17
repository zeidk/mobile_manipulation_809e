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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# moveit
import moveit_commander as mc


class MoveitKairos():

    def __init__(self, node_name='pick_and_place', ns='',
                 robot_description='robot_description'):

        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)
        self.trajectory_pub = rospy.Publisher(
            '/robot/gripper/pos_traj_controller/command',
            JointTrajectory,
            queue_size=10)
        self.locations = {}
        # Joint positions for the arm

        # robot_arm_shoulder_pan_joint
        # robot_arm_shoulder_lift_joint
        # robot_arm_elbow_joint
        # robot_arm_wrist_1_joint
        # robot_arm_wrist_2_joint
        # robot_arm_wrist_3_joint

        name = 'home'
        kairos_arm = [0.85, -0.65, -2.50, -pi/2, pi/2, pi/2]
        self.locations[name] = (kairos_arm)
        
        name = 'table2_grasp'
        kairos_arm = [0.85, -1.15, -0.93, -pi/2, pi/2, pi/2]
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
            if group_name == "ur5_arm":
                group = mc.MoveGroupCommander(
                    group_name, robot_description=ns + '/' + robot_description, 
                    ns=ns)
            elif group_name == "egh_gripper":
                group = mc.MoveGroupCommander(
                    group_name, robot_description=ns + '/' + robot_description, 
                    ns=ns)
                group.set_goal_tolerance(0.05)
            self.groups[group_name] = group
        # self.goto_preset_location("check")
        # self.go_home()

    def cartesian_move(self, group, waypoints):
        group.set_pose_reference_frame("robot_map")
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)
        
    def open_gripper(self):
        jt = JointTrajectory()
        jt.joint_names = ['robot_egh_gripper_finger_left_joint']
        jt.header.stamp = rospy.Time.now()
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.05]
        jtp.velocities = [0.01]
        jtp.time_from_start = rospy.Duration(3.0)
        jt.points.append(jtp)
        self.trajectory_pub.publish(jt)

        # group = self.groups['egh_gripper']
        # finger_position = group.get_current_joint_values()
        # # rospy.loginfo(finger_position)
        # finger_position = [0.02]
        # group.set_joint_value_target(finger_position)
        # group.go()

    def close_gripper(self):
        jt = JointTrajectory()
        jt.joint_names = ['robot_egh_gripper_finger_left_joint']
        jt.header.stamp = rospy.Time.now()
        jtp = JointTrajectoryPoint()
        jtp.positions = [0]
        jtp.velocities = [0.07]
        # jtp.time_from_start = rospy.Duration(1.0)
        jt.points.append(jtp)
        self.trajectory_pub.publish(jt)

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
