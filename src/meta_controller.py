#!/usr/bin/env python

import rospy
import actionlib
from pfvtr.msg import MapRepeaterAction, MapRepeaterResult


map_desc_file = "configs/trav.txt"

class MyROSNode:
    def __init__(self):
        rospy.init_node('meta_control')  # Initialize the ROS node with a unique name

        # Define a publisher
        self.client = actionlib.SimpleActionClient("repeater", MapRepeaterAction)

        # TODO: implement parsing of the description here
        # self.actions = [["map", "map_name"], ["behav", "odom_turn_deg", -90]]
        myfile = open(map_desc_file, "r")
        self.action_strings = myfile.readlines()

    def start_execution()
        for action_string in self.actions_strings:
            action = action_string.split(" ")
            if action[0] == "traversal":
                success = self.traversal(action[1]):
                if not success:
                    raise Exception("Traversal Failed")
            elif action[0] == "behav":
                self.complementary(action[1:])
        rospy.loginfo("Goal reached - quitting meta control.")

    def traversal(map_name):
        rospy.loginfo("Starting traversal of map: " + map_name)
        curr_action = MapRepeaterAction(0, 0, 1, 2, True, map_name)
        self.client.send_goal(curr_action)
        client.wait_for_result()
        rospy.loginfo("Traversal finished!")
        return client.get_result()
        
    def complementary(desc)
        rospy.longinfo("Starting complementary behaviour " + desc[0] + " " + desc[1])
        # TODO: implement controllers for complementary actions
            

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

