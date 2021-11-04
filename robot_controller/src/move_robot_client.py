#!/usr/bin/env python
import rospy
import actionlib
from robot_controller.msg import MoveRobotAction, MoveRobotGoal, StopRobotAction, StopRobotGoal
import time

if __name__ == "__main__":
    rospy.init_node("move_robot_client")
    print("Starting client")
    client_move = actionlib.SimpleActionClient("move_robot", MoveRobotAction)
    client_move.wait_for_server()
    client_stop = actionlib.SimpleActionClient("stop_robot", StopRobotAction)
    client_stop.wait_for_server()
    print("Server found")

    pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
            -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]

    client_move.send_goal(MoveRobotGoal(pose=pose))
    client_move.wait_for_result()


    pose = [0.2154353428673626, -0.26885648388269845, -0.049974561539680856,
            -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    client_move.send_goal(MoveRobotGoal(pose=pose))
    client_move.wait_for_result()


    pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
            -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    client_move.send_goal(MoveRobotGoal(pose=pose))
    time.sleep(1)

    client_stop.send_goal(StopRobotGoal())
    client_stop.wait_for_result()

    print(client_stop.get_result())
    print("done")
'''
    # go to point A
    goal = MoveRobotGoal(pose=[0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                               -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054])
    stop = False
    client.send_goal(goal)
    client.wait_for_result()

    # go to point B
    goal = MoveRobotGoal(pose=[0.5154353428673626, 0.26885648388269845, -0.049974561539680856,
                               -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054, 0])
    client.send_goal(goal)
    client.wait_for_result()

    # go to point A again but send stop before reaching
    goal = MoveRobotGoal(pose=[0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                               -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054, 0])
    client.send_goal(goal)
    time.sleep(3)
    goal = MoveRobotGoal(pose=[0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
                               -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054, 1])
    client.send_goal(goal)
    
    print(client.get_result())
    print("done")
'''
