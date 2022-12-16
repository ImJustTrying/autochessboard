#!/usr/bin/env python3

import sys
import rospy
from board_arm_control.srv import *
from board_arm_control.msg import *

def gen_client(trajectory):
    rospy.wait_for_service('generate_trajectory')
    try:
        generate_trajectory = rospy.ServiceProxy('generate_trajectory', GenerateTrajectory)
        print(JointConfiguration((0,1)))
        for i in range(len(trajectory)):
            trajectory[i] = LatticeCoordinate(trajectory[i])

        resp = generate_trajectory(trajectory)
        return resp.angles
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def path_client(endpoints):
    rospy.wait_for_service('find_path')
    try:
        find_path = rospy.ServiceProxy('find_path', FindLatticePath)
        for i in range(len(endpoints)):
            endpoints[i] = LatticeCoordinate(endpoints[i])

        resp = find_path(endpoints)
        return resp.path
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    endpoints = [(1, 1), (11, 3)]
    print(f"Requesting {endpoints}...")
    res = path_client(endpoints)
    path = []
    for coord in res:
        path.append((coord.point[0], coord.point[1]))
    print(f"Requesting {path}...")
    qs = gen_client(path)
    traj = []
    for q in qs:
        traj.append((q.configuration[0], q.configuration[1]))
    print(f"Result: {len(traj)}")
