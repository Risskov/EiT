#!/usr/bin/env python
import numpy as np
import time
import rospy
import actionlib
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from robot_controller.msg import MoveRobotAction, MoveRobotResult, MoveRobotFeedback, StopRobotAction, StopRobotResult

class ServoControl:
    def __init__(self, controller, receiver):
        self.frequency = 1 / 500
        self.control = controller
        self.receive = receiver
        self.stop = False
        self.velocity = 0.1
        self.acceleration = 0.3
        self.lookAheadTime = 0.1
        self.gain = 600
        # send vel and others in goal message
        self.done = False

    def runTrajectory(self, goal, server):
        self.stop = False
        current_pose = self.receive.getActualTCPPose()[0:3]+[0, 0, 0]
        trajectory = np.linspace(current_pose, goal.pose, 1000)
        pose = trajectory[0]  # Move the robot to the first pose in the trajectory
        self.control.moveL(pose)
        print(pose)
        self.control.zeroFtSensor()
        for point in trajectory:
            if self.stop:
                break
            start_time = time.time()
            self.control.servoL(point, self.velocity, self.acceleration, self.frequency / 2, self.lookAheadTime, self.gain)
            diff = time.time() - start_time  # Ensuring that we do not send commands to the robot too fast
            if diff < self.frequency:
                time.sleep(self.frequency - diff)

            server.publish_feedback(MoveRobotFeedback(force=self.receive.getActualTCPForce()))

        self.control.servoStop()
        server.set_succeeded(MoveRobotResult(self.receive.getActualTCPPose()))

    def move(self, goal, server):
        self.control.moveL(goal.pose, self.velocity, self.acceleration, True)
        while self.rtde_r.getAsyncOperationProgress() >= 0 and not self.stop:
            print("Progress: ", self.rtde_r.getAsyncOperationProgress())
            server.publish_feedback(MoveRobotFeedback(force=self.receive.getActualTCPForce()))
        if self.stop:
            self.control.stopL()
        server.set_succeeded(MoveRobotResult(self.receive.getActualTCPPose()))

    def stopTrajectory(self):
        self.stop = True  # Stopping the servo control when we are done

    def moveTest(self, goal, server):
        self.stop = False
        self.done = self.control.moveL(goal.pose, self.velocity, self.acceleration, True)
        print(self.done)
        while not self.done and not self.stop:
            server.publish_feedback(MoveRobotFeedback(force=self.receive.getActualTCPForce()))
        if self.stop:
            self.control.stopL()
        server.set_succeeded(MoveRobotResult(self.receive.getActualTCPPose()))

class MoveRobot:
    def __init__(self):
        #self.rtde_c = RTDEControl("192.168.1.20", RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_c = RTDEControl("192.168.1.20")
        self.rtde_r = RTDEReceive("192.168.1.20")
        self.servCon = ServoControl(self.rtde_c, self.rtde_r)
        self.move_server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.moveCallback, False)
        self.stop_server = actionlib.SimpleActionServer("stop_robot", StopRobotAction, self.stopCallback, False)
        self.move_server.start()
        self.stop_server.start()
        print("Action server started")

    def moveCallback(self, goal):
        #self.servCon.runTrajectory(goal, self.move_server)
        self.servCon.move(goal, self.move_server)
        #self.servCon.moveTest(goal, self.move_server)

    def stopCallback(self, goal):
        self.servCon.stopTrajectory()
        self.stop_server.set_succeeded(StopRobotResult(True))

if __name__ == "__main__":
    rospy.init_node('move_robot')
    mr = MoveRobot()
    rospy.spin()