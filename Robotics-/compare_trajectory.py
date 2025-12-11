import queue
import mujoco
import numpy as np
import time
import os
import time
import mujoco
import mujoco.viewer
import glfw
import roboticstoolbox as rtb
import spatialmath as sm
import mujoco as mj
import spatialmath as sm
import spatialmath.base as smb
from spatialmath import SE3
from spatialmath.base import trinterp, trnorm

import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend

import matplotlib.pyplot as plt
from typing import List
import roboticstoolbox as rtb
from PIL import Image


from ompl import base as ob
from ompl import geometric as og

from robot import *
from exercises.display_trajectory import display
# from wilbert exercise importing class 
from exercises.exercise_6_sol import StateValidator
from exercises.Point_to_point_trapezoidal_trajectory import program as Trapez
from exercises.RRT_trajectory import program as RR

PI = np.pi

""" 
File to compare both trajectory on a single plot 
"""

if __name__ == "__main__":
    model_path ="scene_final.xml"
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    with mujoco.viewer.launch_passive(model=m, 
                                      data=d, 
                                      key_callback=lambda key: key_queue.put(key)
                                      ) as viewer:
        
        # Home position for the scene
        target_pos = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])  # UR home position
        ur_set_qpos(data=d, q_desired=target_pos)
        hande_ctrl_qpos(data=d, gripper_value=0) # Open gripper


        sim_start = time.time()
        while time.time() - sim_start < 3.0:
            mujoco.mj_step(m, d)
            viewer.sync()

    data_trapez= Trapez(d=d, m=m)
    trapez_tra= np.array([q_pose for q_pose, _ in data_trapez])

    data_RRT = RR(d=d, m=m)
    RRT_tra = np.array([q_pose for q_pose, _ in data_RRT])

    joint_0_trapez = trapez_tra[:,0] # select the first joint 
    joint_0_RRT = RRT_tra[:,0]

    # Plot trajectory profile of joint 0
    fig, axs = plt.subplots(4)
    fig.suptitle('Trajectory profiles of joint 0')

    # trapez
    vel_trapez = np.diff(joint_0_trapez)
    acc_trapez = np.diff(vel_trapez)
    jerk_trapez = np.diff(acc_trapez)

    # RRT
    vel_RRT = np.diff(joint_0_RRT)
    acc_RRT = np.diff(vel_RRT)
    jerk_RRT = np.diff(acc_RRT)

    axs[0].plot(joint_0_trapez, label="Trapez traj")
    axs[0].plot(joint_0_RRT, label="RRT traj")

    axs[1].plot(vel_trapez, label="Trapez traj")
    axs[1].plot(vel_RRT, label="RRT traj")

    axs[2].plot(acc_trapez, label="Trapez traj")
    axs[2].plot(acc_RRT, label="RRT traj")

    axs[3].plot(jerk_trapez, label="Trapez traj")
    axs[3].plot(jerk_RRT, label="RRT traj")

    axs[0].set_ylabel("Position")
    axs[1].set_ylabel("Velocity")
    axs[2].set_ylabel("Accelaration")
    axs[3].set_ylabel("Jerk")

    # plot position of all joints
    fig, axs1 = plt.subplots(3)
    fig.suptitle('Trajectory joint profiles of')
    joint_name = ["soulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    for i in range(3):
        axs1[i].plot(trapez_tra[:,i], label=joint_name[i] + "trapez")
        axs1[i].set_ylabel("Joint" +str(i) + "Position [rd]")
        axs1[i].set_xlabel("Time [s]")
        axs1[i].grid(True)
        axs1[i].legend()

        axs1[i].plot(RRT_tra[:,i], label=joint_name[i] + "RRT")
        axs1[i].set_ylabel("Joint" +str(i) + "Position [rd]")
        axs1[i].set_xlabel("Time [s]")
        axs1[i].grid(True)
        axs1[i].legend()
    plt.savefig("./media/joint_trajectory1_both.pdf", format='pdf')

    fig, axs2 = plt.subplots(3)
    fig.suptitle('Trajectory joint profiles')
    for j in range(3):
        axs2[j].plot(trapez_tra[:,j+3], label=joint_name[j+3]+ "trapez")
        axs2[j].set_ylabel("Joint" +str(j+3) + "Position [rd]")
        axs2[j].grid(True)
        axs2[j].set_xlabel("Time [s]")
        axs2[j].legend()

        axs2[j].plot(RRT_tra[:,j+3], label=joint_name[j+3]+ "RRT")
        axs2[j].set_ylabel("Joint" +str(j+3) + "Position [rd]")
        axs2[j].grid(True)
        axs2[j].set_xlabel("Time [s]")
        axs2[j].legend()
    plt.savefig("./media/joint_trajectory2_both.pdf", format='pdf')

