#!/usr/bin/env python
import rospy
import actionlib
from robot_controller.msg import MoveRobotAction, MoveRobotGoal, StopRobotAction, StopRobotGoal
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class Controller:
    def __init__(self):
        self.states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED",
                       "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
        self.pose_reached = False
        rospy.init_node("controller")
        print("Starting Controller")
        self.client_move = actionlib.SimpleActionClient("move_robot", MoveRobotAction)
        self.client_move.wait_for_server()
        self.client_stop = actionlib.SimpleActionClient("stop_robot", StopRobotAction)
        self.client_stop.wait_for_server()
        print("Server found")

    def sendPose(self, pose):
        self.client_move.send_goal(MoveRobotGoal(pose=pose), done_cb=self.doneCallback, feedback_cb=self.feedbackCallback)
        self.waitForResult()

    def doneCallback(self, state, result):
        self.pose_reached = True
        print(f"State {self.states[state]} reached")
        print("Result: ", result)

    def feedbackCallback(self, feedback):
        force = np.linalg.norm(feedback.force)
        print("Force: ", force)
        if force > 30.0:
            self.client_stop.send_goal(StopRobotGoal())
            #self.pose_reached = True

    def waitForResult(self):
        while not self.pose_reached:
            time.sleep(0.01)
        self.pose_reached = False

    def toBaseFrame(self, point):
        point_a = np.asarray(point)
        transM = [0.13054, -0.24411, -0.03447, 0., 0., -1.17658]
        r = R.from_euler("xyz", transM[3:]).as_matrix()
        T = np.asarray([[1, 0, 0, transM[0]], [0, 1, 0, transM[1]], [0, 0, 1, transM[2]], [0, 0, 0, 1]])
        T[0:3, 0:3] = r
        pose = T @ point_a
        return (pose[0:3]/pose[3]).tolist()+[0, 0, 0]

    def testTrajectory(self, trajectory):
        for point in trajectory:
            print("Sending pose")
            self.sendPose(self.toBaseFrame(point))
        print("Trajectory complete")

if __name__ == "__main__":
    controller = Controller()
    pose0 = [0.4, -0.6, 0.1, 1]
    pose1 = [0.4, 0.6, 0.1, 1]
    traject = [pose1, pose0, pose1, pose0, pose1]
    controller.testTrajectory(traject)
