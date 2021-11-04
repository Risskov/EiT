#!/usr/bin/env python
import rospy
import actionlib
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from robot_controller.msg import MoveRobotAction, MoveRobotResult, StopRobotAction, StopRobotResult

class ServoControl:
    def __init__(self, robot):
        self.frequency = 1 / 500
        self.robot = robot
        self.stop = False
        self.velocity = 0.2
        self.acceleration = 0.5
        self.lookAheadTime = 0.1
        self.gain = 600

    def runTrajectory(self, goal):
        self.stop = False
        current_pose = self.rtde_r.getActualTCPPose()
        trajectory = np.linspace(current_pose, goal.pose, 1000)
        pose = trajectory[0]  # Move the robot to the first pose in the trajectory
        self.robot.moveL(pose)

        for point in trajectory:
            if self.stop:
                break
            start_time = time.time()

            self.robot.servoL(point, self.velocity, self.acceleration, self.frequency / 2, self.lookAheadTime, self.gain)

            diff = time.time() - start_time  # Ensuring that we do not send commands to the robot too fast
            if diff < self.frequency:
                time.sleep(self.frequency - diff)
        self.robot.servoStop()

    def stopTrajectory(self):
        self.stop = True  # Stopping the servo control when we are done

class MoveRobot:
    def __init__(self):

        #self.rtde_c = RTDEControl("192.168.1.20", RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_c = RTDEControl("192.168.1.20")
        self.rtde_r = RTDEReceive("192.168.1.20")
        self.servCon = ServoControl(self.rtde_c)
        self.move_server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.moveCallback, False)
        self.stop_server = actionlib.SimpleActionServer("stop_robot", StopRobotAction, self.servCon.stopTrajectory, False)
        self.move_server.start()
        self.stop_server.start()
        print("Action server started")

    def moveCallback(self, goal):
        self.servCon.runTrajectory(goal)
        self.move_server.set_succeeded(MoveRobotResult(True))

    def stopCallback(self, goal):
        self.servCon.stopTrajectory()
        self.stop_server.set_succeeded()

    '''    
    def callback(self, goal):
        print(goal)
        if goal.stop:
            self.rtde_c.stopL(10)
        else:
            done = self.rtde_c.moveL(goal.pose, 0.25, 0.5, True)
            while not done: pass

        self.server.set_succeeded(MoveRobotResult(True))
        #if goal failed
        #self.server.set_succeeded(MoveRobotResult(False))
    '''
if __name__ == "__main__":
    rospy.init_node('move_robot')
    mr = MoveRobot()
    rospy.spin()