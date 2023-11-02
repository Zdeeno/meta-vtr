#!/usr/bin/env python

import rospy
import actionlib
import math
import numpy as np
from pfvtr.msg import MapRepeaterAction, MapRepeaterResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


P = 3
THRESHOLD = 0.05
MAX_CMD = 1.0
map_desc_file = "configs/trav.txt"
odometry_topic = "blah_blah"
control_topic = "blah_blah"


class MyROSNode:
    def __init__(self):
        rospy.init_node('meta_control')  # Initialize the ROS node with a unique name

        # Define a publisher
        self.comp = False
        self.comp_diff = None
        self.comp_goal = None
        self.client = actionlib.SimpleActionClient("repeater", MapRepeaterAction)
        self.odom_sub = rospy.Subscriber(odometry_topic, Odometry, odom_cb)
        self.control_pub = rospy.Publisher(control_topic, Twist)

        # TODO: implement parsing of the description here
        # self.actions = [["map", "map_name"], ["behav", "odom_turn_deg", -90]]
        myfile = open(map_desc_file, "r")
        self.action_strings = myfile.readlines()

    def start_execution(self)
        for action_string in self.actions_strings:
            action = action_string.split(" ")
            if action[0] == "traversal":
                success = self.traversal(action[1]):
                if not success:
                    raise Exception("Traversal Failed")
            elif action[0] == "behav":
                self.complementary(action[1:])
        rospy.loginfo("Goal reached - quitting meta control.")

    def traversal(self, map_name):
        rospy.loginfo("Starting traversal of map: " + map_name)
        curr_action = MapRepeaterAction(0, 0, 1, 2, True, map_name)
        self.client.send_goal(curr_action)
        client.wait_for_result()
        rospy.loginfo("Traversal finished!")
        return client.get_result()
        
    def complementary(self, desc)
        rospy.longinfo("Starting complementary behaviour " + desc[0] + " " + desc[1])
        self.comp_diff = float(desc[1])
        self.comp = True
        while comp:
            rospy.sleep(1.0)
            
    def odom_cb(self, msg):
        if not self.comp:
            # Not using this behaviour
            return
        curr_or = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        if self.comp_goal is None:
            # Init goal
            self.comp_goal = curr_or + np.deg2rad(self.comp_diff)
        curr_diff = angular_diff(self.comp_goal, curr_or)
        if abs(curr_diff) <= THRESHOLD:
            # goal reached
            self.comp = False
            self.comp_goal = None
            self.comp_diff = None
        # control robot
        control_cmd = min(P * curr_diff, MAX_CMD)
        msg_cmd = Twist()
        msg_cmd.angular.z = control_cmd
        self.control_pub.publish(msg_cmd)
        
        
    def angular_diff(self, x, y):
        a = (x - y) % 2*math.pi
        b = (y - x) % 2*math.pi
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

