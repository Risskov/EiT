#!/usr/bin/env python
import rospy
import actionlib
from rtde_control import RTDEControlInterface as RTDEControl
from robot_controller.msg import MoveRobotAction, MoveRobotResult


class MoveRobot:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.callback, False)
        self.rtde_c = RTDEControl("192.168.1.20", RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")
        self.server.start()
        print("Action server started")

    def callback(self, goal):
        print(goal)
        if goal[6] == 1:
            self.rtde_c.stopJ()
        elif goal[6] == 0:
            self.rtde_c.moveL(goal[0:6], 0.25, 0.5, True)
            self.server.set_succeeded(MoveRobotResult(True))
            #if goal failed
            #self.server.set_succeeded(MoveRobotResult(False))

if __name__ == "__main__":
    rospy.init_node('move_robot')
    mr = MoveRobot()
    rospy.spin()