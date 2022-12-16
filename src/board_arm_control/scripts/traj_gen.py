#!/usr/bin/env python3

import numpy as np
import rospy
from numpy import arctan2, sin, cos, arccos
from scipy.linalg import pinv
from math import ceil
from board_arm_control.srv import GenerateTrajectory, GenerateTrajectoryResponse
from board_arm_control.msg import JointConfiguration

l  = 7.125 # link length
dt = 0.01  # time delta
# 1 rpm = 2pi/60 rad/s 
v_limit = 100 * 2 * np.pi / 60
a_limit = 50

# Return positions of the ends of links for given configurations
def fk(q_hist):
    N, _ = q_hist.shape
    ee = np.empty((N, 2, 2))
    angle_sum = q_hist[:,0] + q_hist[:,1]
    """
    Each matrix is as follows
    [[ x_1, y_1 ]
     [ x_2, y_2 ]]
    """
    ee[:,0,0] = l * cos(q_hist[:,0])
    ee[:,0,1] = l * sin(q_hist[:,0])
    ee[:,1,0] = ee[:,0,0] + l * cos(angle_sum)
    ee[:,1,1] = ee[:,0,1] + l * sin(angle_sum)
    return ee

# Given list of end effector positions, returns corresponding joint positions
def ik(x_hist):
    norm_squared = x_hist[:,0] ** 2 + x_hist[:,1] ** 2
    q1 = arccos(norm_squared / (2 * l**2) - 1)
    q0 = arctan2(x_hist[:,1], x_hist[:,0]) - q1 / 2.0
    return np.transpose(np.array([q0, q1]))

# compute joint velocities given end effector velocities
def ivk(v_hist, q_hist):
    J = lambda q: np.array(
            [[-l * sin(q[0]) - l * sin(q[0] + q[1]), -l * sin(q[0] + q[1])],
             [l * cos(q[0]) + l * cos(q[0] + q[1]), l * cos(q[0] + q[1])]])
    dq_hist = np.empty(v_hist.shape)
    for i in range(v_hist.shape[0]):
        dq_hist[i] = pinv(J(q_hist[i])) @ v_hist[i]
    return dq_hist


# Refer to Modern Robotics section 9.2.1 Straight-Line Paths
def simulate(p_start, p_end, v_limit, a_limit, T=1.0):
    # Straight line path as a function of time scale s(t) in [0,1]
    p_s = lambda s: np.array([p_start + k * (p_end - p_start) for k in s])
    dp_s = lambda ds: np.array([k * (p_end - p_start) for k in ds])
    ddp_s = lambda dds: np.array([k * (p_end - p_start) for k in dds])

    # Using quintic polynomial time scaling
    # Initial conditions: s(0) = ds(0) = dds(0) = ds(T) = dds(T) = 0, s(T) = 1
    s_t =   lambda t: 10/T**3 * t**3 -  15/T**4 * t**4 +   6/T**5 * t**5
    ds_t =  lambda t: 30/T**3 * t**2 -  60/T**4 * t**3 +  30/T**5 * t**4
    dds_t = lambda t: 60/T**3 * t    - 180/T**4 * t**2 + 120/T**5 * t**3

    t = np.linspace(0, T, num=ceil(T/dt))
    ps = p_s(s_t(t))
    dps = dp_s(ds_t(t))
    ddps = ddp_s(dds_t(t))
    qs = ik(ps)
    dqs = ivk(dps, qs)
    # approximate derivative of joint velocity
    ddqs = np.empty(dqs.shape)
    ddqs[0] = 0
    for i in range(1, dqs.shape[0]):
        ddqs[i] = (dqs[i] - dqs[i - 1]) / dt

    # if time to complete motion violates velocity or acceleration limits,
    # recompute new time that respects these limits
    vmax = np.max(dqs)
    amax = np.max(ddqs)
    if vmax > v_limit or amax > a_limit:
        T *= max(vmax / v_limit, np.sqrt(amax / a_limit))
        print(f'new time = {T}')
        t = np.linspace(0, T, num=ceil(T/dt))
        ps = p_s(s_t(t))
        dps = dp_s(ds_t(t))
        ddps = ddp_s(dds_t(t))
        qs = ik(ps)
        dqs = ivk(dps, qs)
        # approximate derivative of joint velocity
        ddqs = np.empty(dqs.shape)
        ddqs[0] = 0
        for i in range(1, dqs.shape[0]):
            ddqs[i] = (dqs[i] - dqs[i - 1]) / dt

    return (ps, dps, ddps, qs, dqs, ddqs)


def handler(req):
    angles = []
    print(req.trajectory[0].point)
    for i in range(len(req.trajectory) - 1):
        print(req.trajectory[i + 1].point)
        _, _, _, qs, _, _ = simulate(
                np.array(req.trajectory[i].point),
                np.array(req.trajectory[i + 1].point),
                v_limit,
                a_limit)
        for q in qs:
            angles.append(JointConfiguration((q[0], q[1])))
    return GenerateTrajectoryResponse(angles)

def traj_gen_server():
    rospy.init_node("traj_gen_server")
    s = rospy.Service("generate_trajectory", GenerateTrajectory, handler)
    print("Ready to generate trajectories")
    rospy.spin()

if __name__ == "__main__":
    traj_gen_server()
