#!/usr/bin/env python

import rospy
import actionlib
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import os
from pfvtr.msg import MapRepeaterAction, MapRepeaterResult, MapRepeaterGoal

P = 1
THRESHOLD = 0.05
MAX_CMD = 1.0
MIN_CMD = 0.1
file_dir = os.path.dirname(os.path.realpath(__file__))
map_desc_file = os.path.join(file_dir, "../configs/sim_test.txt")
# odometry_topic = "/robot1/odometry"
# control_topic = "/robot1/velocity_reference"


class MyROSNode:
    def __init__(self):
        rospy.init_node('meta_control')  # Initialize the ROS node with a unique name

        # Define a publisher
        self.comp = False
        self.comp_diff = None
        self.comp_goal = None

        odometry_topic = rospy.get_param("~odom_topic")
        control_topic = rospy.get_param("~cmd_vel_pub")

        self.client = actionlib.SimpleActionClient("/pfvtr/repeater", MapRepeaterAction)
        self.client.wait_for_server()
        self.odom_sub = rospy.Subscriber(odometry_topic, Odometry, self.odom_cb)
        self.control_pub = rospy.Publisher(control_topic, Twist, queue_size=10)

        # self.actions = [["map", "map_name"], ["behav", "odom_turn_deg", -90]]
        myfile = open(map_desc_file, "r")
        self.action_strings = myfile.readlines()

    def start_execution(self):
        for action_string in self.action_strings:
            action = action_string.split(" ")
            if action[0] == "map":
                success = self.traversal(action[1], int(action[2]))
                if not success:
                    raise Exception("Traversal Failed")
            elif action[0] == "behav":
                self.complementary(action[1:])
        rospy.loginfo("All tasks fulfilled - quitting meta control.")

    def traversal(self, map_name, lookaround):
        rospy.loginfo("Starting traversal of map: " + map_name)
        curr_action = MapRepeaterGoal(startPos=0.0, endPos=0.0, traversals=0, nullCmd=True, imagePub=lookaround,
                                      useDist=True, mapName=map_name)
        self.client.send_goal(curr_action)
        rospy.loginfo("Goal sent.")
        self.client.wait_for_result()
        rospy.loginfo("Traversal finished!")
        return self.client.get_result()

    def complementary(self, desc):
        rospy.loginfo("Starting complementary behaviour " + desc[0] + " " + desc[1])
        self.comp_diff = float(desc[1])
        self.comp = True
        while self.comp:
            rospy.sleep(1.0)

    def odom_cb(self, msg):
        if not self.comp:
            # Not using this behaviour
            return
        curr_or = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[-1]
        if self.comp_goal is None:
            # Init goal
            self.comp_goal = curr_or + np.deg2rad(self.comp_diff)
            rospy.logwarn("goal: " + str(self.comp_goal))
            rospy.logwarn("curr: " + str(curr_or))
            rospy.logwarn("target: " + str(np.deg2rad(self.comp_diff)))
        curr_diff = self.angular_diff(self.comp_goal, curr_or)
        if abs(curr_diff) <= THRESHOLD:
            rospy.loginfo("Complementary goal reached!")
            self.control_pub.publish(Twist())
            self.comp = False
            self.comp_goal = None
            self.comp_diff = None
            return
        # control robot
        rospy.logwarn("goal: " + str(self.comp_goal) + ", curr: " + str(curr_or) + ", diff: " + str(curr_diff))
        control_cmd = -np.sign(curr_diff) * max(min(abs(P * curr_diff), MAX_CMD), MIN_CMD)
        msg_cmd = Twist()
        msg_cmd.angular.z = control_cmd
        rospy.logwarn(control_cmd)
        self.control_pub.publish(msg_cmd)
        return


    def angular_diff(self, x, y):
        a = (x - y) % (2*math.pi)
        b = (y - x) % (2*math.pi)
        return -a if a < b else b

    def run(self):
        # Main loop to keep the node running
        self.start_execution()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MyROSNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("Process interrupted!")
