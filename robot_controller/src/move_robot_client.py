#!/usr/bin/env python
import rospy
import actionlib
from robot_controller.msg import MoveRobotAction, MoveRobotGoal, StopRobotAction, StopRobotGoal
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from robot_controller import Segmentation_WithRW as sw
from robot_controller import pathplanning as pp

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
        self.timeSinceStop = 10000
        self.forcehistory = []
        self.runningMean = 0
        print("Server found")

    def sendPose(self, pose):
        self.client_move.send_goal(MoveRobotGoal(pose=pose), done_cb=self.doneCallback, feedback_cb=self.feedbackCallback)
        self.waitForResult()

    def doneCallback(self, state, result):
        self.pose_reached = True
        print(f"State {self.states[state]} reached")
        print("Result: ", result)

    def feedbackCallback(self, feedback):
        force = np.linalg.norm(feedback.force[0:2])
        self.runningMean = self.runningMean*0.8+force*0.2

        self.forcehistory.append(feedback.force)
        if len(self.forcehistory) > 500:
            self.forcehistory.pop(0)

        #print("Force: ", force)
        if self.runningMean > 20000.0 and self.timeSinceStop > 10:
            self.client_stop.send_goal(StopRobotGoal())
            self.timeSinceStop = 0
            self.pose_reached = True
        self.timeSinceStop += 1

    def waitForResult(self):
        while not self.pose_reached:
            time.sleep(0.01)
        self.pose_reached = False

    def toBaseFrame(self, point):
        point = point+[1]
        point_a = np.asarray(point)
        transM = [0.13054, -0.24411, -0.03447, 0., 0., -1.17658]
        r = R.from_euler("xyz", transM[3:]).as_matrix()
        T = np.asarray([[1, 0, 0, transM[0]], [0, 1, 0, transM[1]], [0, 0, 1, transM[2]], [0, 0, 0, 1]])
        T[0:3, 0:3] = r
        pose = T @ point_a
        return (pose[0:3]/pose[3]).tolist()#+[0, 0, 0]

    def toTableFrame(self, point):
        point = point.tolist()+[1]
        point_a = np.asarray(point)
        transM = [0.13054, -0.24411, -0.03447, 0., 0., -1.17658]
        r = R.from_euler("xyz", transM[3:]).as_matrix()
        T = np.asarray([[1, 0, 0, transM[0]], [0, 1, 0, transM[1]], [0, 0, 1, transM[2]], [0, 0, 0, 1]])
        T[0:3, 0:3] = r

        print(point_a)
        pose = np.linalg.inv(T) @ point_a
        return (pose[0:3] / pose[3]).tolist()  # +[0, 0, 0]

    def runTrajectory(self, trajectory):
        for point in trajectory:
            print("Sending pose")
            #self.sendPose(self.toBaseFrame(point))
            self.sendPose(self.toBaseFrame(point)+[0,0,0])
        print("Trajectory complete")

if __name__ == "__main__":
    controller = Controller()
    init_point = [0.200, 0.300, 0.600]
    controller.runTrajectory([init_point])
    #print("Init base: ", controller.toBaseFrame(init_point))
    #exit()
    seg = sw.Segmentation()
    points = seg.run()

    for i , point in enumerate(points):
        points[i] = controller.toTableFrame(point)


    print("Points relative to table ", points)
    #radius = 0.017
    #points = np.asarray([[0.100-radius, 0.290-radius, 0.02], [0.100-radius, 0.500+radius, 0.02], [0.320+radius, 0.500+radius, 0.02], [0.320+radius, 0.290-radius, 0.02]])
    pathPlaner = pp.sidePathPlaner(points, resolution=0.05, penDist=0.01)

    pathPlaner.plot2D()
    traject = pathPlaner.path3D.tolist()
    topPlaner = pp.probePathTop(points, zOffset=0.05, intRes=[0.05, 0.05], penDist=0.01, toolOffset=0.05)
    #topPlaner.plot()
    traject2 = topPlaner.path.tolist()
    print(traject)

    controller.runTrajectory(traject)
    controller.runTrajectory(traject2)
    controller .runTrajectory([init_point])
