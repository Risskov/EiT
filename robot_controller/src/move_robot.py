#!/usr/bin/env python

import rospy
import actionlib
from rtde_control import RTDEControlInterface as RTDEControl
from robot_controller.msg import MoveRobotAction


class MoveRobot:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.callback, False)
        self.rtde_c = RTDEControl("192.168.1.20", RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.server.start()

    def callback(self, goal):
        print(goal)
        #self.rtde_c.moveL(goal)

if __name__ == "__main__":
    rospy.init_node('move_robot')
    mr = MoveRobot()
    rospy.spin()