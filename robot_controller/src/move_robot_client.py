#!/usr/bin/env python
import rospy
import actionlib
from robot_controller.msg import MoveRobotAction, MoveRobotGoal, StopRobotAction, StopRobotGoal
import time
import numpy as np

class Controller:
    def __init__(self):
        self.states = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED",
                       "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
        self.pose_reached = False
        rospy.init_node("move_robot_client")
        print("Starting client")
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
        #if feedback > 5:
        #    self.pose_reached = True

    def waitForResult(self):
        while not self.pose_reached:
            time.sleep(0.01)
        self.pose_reached = False

    def testTrajectory(self):
        pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
        self.sendPose(pose)

        pose = [0.3154353428673626, -0.26885648388269845, -0.049974561539680856,
                -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
        self.sendPose(pose)

        pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
        self.client_move.send_goal(MoveRobotGoal(pose=pose), done_cb=self.doneCallback)
        time.sleep(1)
        self.client_stop.send_goal(StopRobotGoal())
        self.client_stop.wait_for_result()

        print(self.client_stop.get_result())
        print("done")

if __name__ == "__main__":
    controller = Controller()
    controller.testTrajectory()

