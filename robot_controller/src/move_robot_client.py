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
        if force > 7:
            self.pose_reached = True

    def waitForResult(self):
        while not self.pose_reached:
            time.sleep(0.01)
        self.pose_reached = False

    def testTrajectory(self, trajectory):
        '''
        pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
        self.sendPose(pose)

        pose = [0.2154353428673626, -0.26885648388269845, -0.049974561539680856,
                -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
        self.sendPose(pose)

        pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
        self.client_move.send_goal(MoveRobotGoal(pose=pose), done_cb=self.doneCallback)
        time.sleep(1)
        self.client_stop.send_goal(StopRobotGoal())
        self.client_stop.wait_for_result()
        '''
        for traj in trajectory:
            self.sendPose(traj)
        #print(self.client_stop.get_result())
        print("done")

def toBaseFrame(point):
    pointnp = np.asarray(point)
    transM = [0.13054, -0.24411, -0.03447, 0., 0., -1.17658]
    r = R.from_euler("xyz", transM[3:]).as_matrix()
    t = np.asarray([[1, 0, 0, transM[0]], [0, 1, 0, transM[1]], [0, 0, 1, transM[2]], [0, 0, 0, 1]])
    t[0:3, 0:3] = r
    pose = t @ pointnp
    return (pose[0:3]/pose[3]).tolist()+[0, 3.1415, 0]

if __name__ == "__main__":
    controller = Controller()
    pose0 = [0.4, -0.6, 0.1, 1]
    pose1 = [0.4, 0.6, 0.1, 1]
    point0BF = toBaseFrame(pose0)
    point1BF = toBaseFrame(pose1)
    print(point1BF)
    #pose0 = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
    #         -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    #pose1 = [0.1154353428673626, -0.26885648388269845, -0.049974561539680856,
    #         -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    traject = [point1BF, point0BF, point1BF, point0BF, point1BF]
    controller.testTrajectory(traject)

