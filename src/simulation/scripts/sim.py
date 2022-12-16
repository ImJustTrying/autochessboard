import matplotlib.pyplot as plt
import numpy as np
from numpy import arctan2, sin, cos, arccos, arctan
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from scipy.linalg import pinv
from math import ceil

l  = 7.125 # link length
dt = 0.01 # time delta
fps = 60   # frames per second of animation
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


def generate_animation(x_hist):
    fig = plt.figure(figsize=(10,10))
    ax = plt.subplot(1,1,1)   
    link1, = ax.plot([], [], 'r', lw=10, alpha=0.4)
    link2, = ax.plot([], [], 'b', lw=10, alpha=0.4)
    ax.add_patch(Rectangle((-11, -11), 22, 22, fill=False))

    # create matplotlib shit and set aspect ratio to 1
    ax.set_xlim((-15, 15))
    ax.set_ylim((-15, 15))
    ax.set_aspect(1)
    txt_title = ax.text(-2, 14, '')

    array_length = x_hist.shape[0]
    # time between frames in milliseconds
    interval = 1000/fps
    interval_sec = interval/1000
    # total animation time in seconds
    total_run_time = array_length * dt
    num_frames = ceil(total_run_time / interval_sec)
    # pre compute frame indices into x_hist and simulation times
    frames = np.around(np.arange(num_frames) * array_length / num_frames).astype(int)
    times = np.arange(num_frames) * interval_sec
    frame_times = list(zip(frames, times))
    print(f'number of frames: {num_frames}')

    def draw_frame(frame):
        keypts = x_hist[frame[0]]

        x1 = keypts[0, 0]
        x2 = keypts[1, 0]
        y1 = keypts[0, 1]
        y2 = keypts[1, 1]
        link1.set_data([0, x1], [0, y1])
        link2.set_data([x1, x2], [y1, y2])
        txt_title.set_text(f't = {frame[1]:.2f} sec')
        return link1, link2, txt_title,

    anim = FuncAnimation(
            fig,
            draw_frame,
            frames=frame_times,
            interval=interval,
            repeat_delay=2000,
            blit=True)
    return anim


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

    plt.subplot(2, 3, 1)
    plt.plot(t, ps)
    plt.subplot(2, 3, 2)
    plt.plot(t, dps)
    plt.subplot(2, 3, 3)
    plt.plot(t, ddps)
    plt.subplot(2, 3, 4)
    plt.plot(t, qs)
    plt.subplot(2, 3, 5)
    plt.plot(t, dqs)
    plt.subplot(2, 3, 6)
    plt.plot(t, ddqs)
    plt.show()

    return (ps, dps, ddps, qs, dqs, ddqs)


hist1 = simulate(np.array([-10,1]), np.array([10,1]), v_limit, a_limit)[0]
hist2 = simulate(np.array([10,1]), np.array([10,10]), v_limit, a_limit)[0]
hist3 = simulate(np.array([10,10]), np.array([6,10]), v_limit, a_limit)[0]
hist4 = simulate(np.array([6,10]), np.array([6,-10]), v_limit, a_limit)[0]
hist = np.concatenate((hist1, hist2, hist3, hist4), axis=0)
#np.append(hist, simulate(np.array([2,-15]), np.array([3,-15]), v_limit, a_limit))
anim = generate_animation(fk(ik(hist)))
plt.show()
