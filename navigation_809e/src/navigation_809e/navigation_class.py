#!/usr/bin/env python

import rospy
import sys
import tf
import yaml
import copy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from navigation_809e.util import compute_distance
from tf.transformations import euler_from_quaternion
from math import radians, atan2, pi
from sensor_msgs.msg import LaserScan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Navigation():
    """
    A controller class to drive a turtlebot in Gazebo.
    """

    def __init__(self, rate=10):
        rospy.init_node('pick_and_place', anonymous=True)
        rospy.loginfo('Press Ctrl c to exit')
        self._velocity_publisher = rospy.Publisher(
            'cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        # MoveBase stuff
        self._check = [-6.3703, 2.2929, 0, 0, 0, -0.6866, 0.727]
        self._home = [0, 0, 0, 0, 0, 0, 1]
        self._pre_table2 = [-3, 3.8, 0, 0, 0, 0, 1]
        self._table2 = [-1.4, 3.8, 0, 0, 0, 0, 1]
        self.client = actionlib.SimpleActionClient(
            'robot/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "robot_map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self._rate = rospy.Rate(rate)
        self._robot_name = 'waffle'
        self._velocity_msg = Twist()
        self._kp_linear = 0.3
        self._kp_angular = 0.3
        self._velocity_msg.linear.x = 0.3
        self._velocity_msg.angular.z = 0.3
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._initial_orientation = None
        self._yaw = None
        # start position of the robot
        self._initial_x_pos = None
        self._initial_y_pos = None

        # set up a tf listener to retrieve transform between the robot and the world
        self._tf_listener = tf.TransformListener()
        # parent frame for the listener
        self._parent_frame = 'odom'
        # child frame for the listener
        self._child_frame = 'base_footprint'

    def odom_callback(self, msg):
        """
        Callback function for the Topic odom

        Args:
            msg (nav_msgs/Odometry): Odometry message
        """
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.current_x_pos = msg.pose.pose.position.x
        self.current_y_pos = msg.pose.pose.position.y
        self.current_orientation = euler_from_quaternion(quaternion)

    def get_transform(self):
        """
        Get the current pose of the robot in the world frame.
        """
        try:
            now = rospy.Time.now()
            self._tf_listener.waitForTransform(self._parent_frame,
                                               self._child_frame,
                                               now,
                                               rospy.Duration(5))
            (trans, rot) = self._tf_listener.lookupTransform(
                self._parent_frame, self._child_frame, now)
            self.current_x_pos = trans[0]
            self.current_y_pos = trans[1]
            self.current_orientation = rot
            rospy.loginfo("odom: ({},{}), {}".format(
                self.current_x_pos,
                self.current_y_pos,
                self.current_orientation[2]))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")

    def move_to(self, goal_x=None, goal_y=None, goal_yaw=None, spot=None):

        chosen_spot = []
        if spot is not None:
            if spot == "home":
                chosen_spot = copy.deepcopy(self._home)
            elif spot == "check":
                chosen_spot = copy.deepcopy(self._check)
            elif spot == "pre_table2":
                chosen_spot = copy.deepcopy(self._pre_table2)
            elif spot == "table2":
                chosen_spot = copy.deepcopy(self._table2)
            self.goal.target_pose.pose.position.x = chosen_spot[0]
            self.goal.target_pose.pose.position.y = chosen_spot[1]
            self.goal.target_pose.pose.orientation.x = chosen_spot[3]
            self.goal.target_pose.pose.orientation.y = chosen_spot[4]
            self.goal.target_pose.pose.orientation.z = chosen_spot[5]
            self.goal.target_pose.pose.orientation.w = chosen_spot[6]
        else:
            self.goal.target_pose.pose.position.x = goal_x
            self.goal.target_pose.pose.position.y = goal_y
            quaternion = tf.transformations.quaternion_from_euler(
                0, 0, goal_yaw)
            # type(pose) = geometry_msgs.msg.Pose
            self.goal.target_pose.pose.orientation.x = quaternion[0]
            self.goal.target_pose.pose.orientation.y = quaternion[1]
            self.goal.target_pose.pose.orientation.z = quaternion[2]
            self.goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def run(self, linear, angular):
        """
        Publish linear and angular velocities to cmd_vel Topic.

        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")

    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        # get the action to perform
        # drive straight, rotate, or go to goal
        action_name = rospy.get_param("~action")

        if action_name == "drive":
            distance = rospy.get_param("~distance")
            if distance > 0:
                self.go_straight_tf(distance, True)
            elif distance < 0:
                self.go_straight_tf(distance, False)
            else:
                rospy.logerr("Distance not provided")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "rotate":
            angle = None
            angle = rospy.get_param("~angle")
            if angle is not None:
                self.rotate(angle)
            else:
                rospy.logerr("Angle not provided")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "goal":
            x = None
            y = None
            x = rospy.get_param("~x")
            y = rospy.get_param("~y")
            if x is not None and y is not None:
                self.go_to_goal(x, y)
            else:
                rospy.logerr("x or y is missing")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "moveto":
            x = None
            y = None
            yaw = None
            spot = None
            if rospy.has_param('~x'):
                x = rospy.get_param("~x")
            if rospy.has_param('~y'):
                y = rospy.get_param("~y")
            if rospy.has_param('~yaw'):
                yaw = rospy.get_param("~yaw")
            if rospy.has_param('~spot'):
                spot = rospy.get_param("~spot")

            if x is not None and y is not None and yaw is not None:
                self.move_to(goal_x=x, goal_y=y, goal_yaw=yaw, spot=None)
            if spot is not None:
                self.move_to(goal_x=None, goal_y=None,
                             goal_yaw=None, spot=spot)
            else:
                rospy.logerr("x, y, yaw, or spot is missing")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "broadcast":
            while not rospy.is_shutdown():
                self.broadcast()
        elif action_name == "yaml":
            self.update_yaml()
        else:
            rospy.logerr("Unknown action")
            rospy.on_shutdown(self.myhook)
            sys.exit(1)

    def broadcast(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((0.1, 0.2, 1),
                         tf.transformations.quaternion_from_euler(0, 0, 3.14),
                         rospy.Time.now(),
                         "marker",
                         "camera_rgb_optical_frame")

    def update_yaml(self):
        fname = "output/test.yaml"
        stream = open(fname, 'r')
        data = yaml.load(stream)
        data['aruco_marker_0']['xyz'] = [0, 10, 0]
        data['aruco_marker_0']['rpy'] = [1, 1, 1]
        with open(fname, 'w') as yaml_file:
            yaml_file.write(yaml.dump(data, default_flow_style=False))
