#!/usr/bin/env python
import rospy
import actionlib
from robot_controller.msg import MoveRobotAction, MoveRobotGoal
import time

if __name__ == "__main__":
    rospy.init_node("move_robot_client")
    client = actionlib.SimpleActionClient("move_robot", MoveRobotAction)
    client.wait_for_server()

    pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
            -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    stop = False
    client.send_goal(MoveRobotGoal(pose=pose, stop=stop))
    client.wait_for_result()


    pose = [0.5154353428673626, -0.16885648388269845, -0.049974561539680856,
            -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    client.send_goal(MoveRobotGoal(pose=pose, stop=stop))
    client.wait_for_result()


    pose = [0.5154353428673626, -0.26885648388269845, -0.049974561539680856,
            -0.0011797094876681562, 3.1162457375630024, 0.038842380117300054]
    client.send_goal(MoveRobotGoal(pose=pose, stop=stop))
    time.sleep(2)
    stop = True
    client.send_goal(MoveRobotGoal(pose=pose, stop=stop))
    client.wait_for_result()
    print(client.get_result())
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
