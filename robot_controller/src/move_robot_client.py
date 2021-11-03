#!/usr/bin/env python
import rospy
import actionlib
from robot_controller.msg import MoveRobotAction, MoveRobotGoal

if __name__ == "__main__":
    rospy.init_node("move_robot_client")
    client = actionlib.SimpleActionClient("move_robot", MoveRobotAction)
    client.wait_for_server()

    goal = MoveRobotGoal(pose=[1, 1, 1, 1, 1, 1])
    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_result())
    print("done")