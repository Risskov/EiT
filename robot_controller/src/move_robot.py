#!/usr/bin/env python
import numpy as np
import time
import rospy
import actionlib
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from robot_controller.msg import MoveRobotAction, MoveRobotResult, MoveRobotFeedback, StopRobotAction, StopRobotResult
import signal
from threading import Thread
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R

class MoveRobot:
    def __init__(self):
        #self.rtde_c = RTDEControl("192.168.1.20", RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_c = RTDEControl("192.168.1.20", RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_r = RTDEReceive("192.168.1.20")
        self.move_server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.moveCallback, False)
        self.stop_server = actionlib.SimpleActionServer("stop_robot", StopRobotAction, self.stopCallback, False)
        self.move_server.start()
        self.stop_server.start()
        self.stop = False
        self.velocity = 0.1
        self.acceleration = 0.5
        self.rtde_c.zeroFtSensor()
        print("Action server started")
        self.pub = Thread(target=self.publishTf)
        self.pub.start()

    def publishTf(self):
        pub = rospy.Publisher("/tf", TFMessage, queue_size=1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #if threadevent.is_set(): return
            rate.sleep()
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "base_frame"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "tcp"

            pose = self.rtde_r.getActualTCPPose()
            r = R.from_rotvec(pose[3:6])
            quat = r.as_quat()
            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            tfm = TFMessage([t])
            pub.publish(tfm)

    def moveCallback(self, goal):
        self.stop = False
        self.rtde_c.moveL(goal.pose, self.velocity, self.acceleration, True)
        while not self.rtde_c.isSteady() and not self.stop:
            pass
            self.move_server.publish_feedback(MoveRobotFeedback(force=self.rtde_r.getActualTCPForce()))
        if self.stop:
            self.rtde_c.stopL(0.5)
            print("Robot stopped")
        self.move_server.set_succeeded(MoveRobotResult(self.rtde_r.getActualTCPPose()))

    def stopCallback(self, goal):
        self.stop = True
        self.stop_server.set_succeeded(StopRobotResult(True))

    def exitGracefully(self, signum, frame):
        self.rtde_c.stopScript()
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        exit("Closing program")

if __name__ == "__main__":
    rospy.init_node('move_robot')
    mr = MoveRobot()
    signal.signal(signal.SIGINT, mr.exitGracefully)
    rospy.spin()