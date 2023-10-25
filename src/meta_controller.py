#!/usr/bin/env python

import rospy
import actionlib
from pfvtr.msg import MapRepeaterAction, MapRepeaterResult

class MyROSNode:
    def __init__(self):
        rospy.init_node('meta_control')  # Initialize the ROS node with a unique name

        # Define a publisher
        self.client = actionlib.SimpleActionClient("repeater", MapRepeaterAction)

        # TODO: implement parsing of the description here
        self.actions = [["traversal", "map_name"], ["odometry", -90]]

    def start_execution()
        for action in self.actions:
            if action[0] == "traversal":
                success = self.traversal(action[1]):
                if not success:
                    raise Exception("Traversal Failed")
            else:
                self.complementary(action)

    def traversal(map_name):
        rospy.loginfo("Starting traversal of map: " + map_name)
        curr_action = MapRepeaterAction(0, 0, 1, 2, True, map_name)
        self.client.send_goal(curr_action)
        client.wait_for_result()
        rospy.loginfo("Traversal finished!")
        return client.get_result()
        
    def complementary(description)
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

