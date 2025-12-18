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
Author:DELIN Matthieu 
File to compare both methods on their trajectory, speed, acceleration and forward kinematics. The reuslt are saved in PDF isnide the folder media. 
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
    
    T_tra = np.linspace(0,0.002*np.shape(trapez_tra)[0],np.shape(trapez_tra)[0])
    T_RRT = np.linspace(0,0.002*np.shape(RRT_tra)[0],np.shape(RRT_tra)[0])

    robot = UR5robot(data=d, model=m)
    joint_name = ["soulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    # forward Kinematics computation
    cartesian_trapez = []
    cartesian_RRT = []
    for i in trapez_tra:
        frame_trapez = robot.robot_ur5.fkine(i).t
        cartesian_trapez.append(frame_trapez)
    for j in RRT_tra:
        frame_rrt = robot.robot_ur5.fkine(j).t
        cartesian_RRT.append(frame_rrt)

    #  converison to plot easily
    cartesian_RRT = np.array(cartesian_RRT)
    print("frame_rrt",frame_rrt)
    print("Diretc kinematics",np.shape(cartesian_RRT), cartesian_RRT[-1])
    cartesian_trapez = np.array(cartesian_trapez)

    fig, axs = plt.subplots(3)
    fig.suptitle("Direct Kinematics from both methods for box case")
    # Z regarding X
    axs[0].plot(T_tra,cartesian_trapez[:,0], label="Trapez")
    axs[0].plot(T_RRT,cartesian_RRT[:,0], label="RRT")
    axs[0].set_ylabel("X Axis [m]")
    axs[0].set_xlabel("Time [s]")
    axs[0].grid(True)
    axs[0].legend()

    # Z regarding Y
    axs[1].plot(T_tra,cartesian_trapez[:,1], label="Trapez")
    axs[1].plot(T_RRT,cartesian_RRT[:,1], label="RRT")
    axs[1].set_ylabel("Y Axis [m]")
    axs[1].set_xlabel("Time [s]")
    axs[1].grid(True)
    axs[1].legend()

    # X regarding Y
    axs[2].plot(T_tra,cartesian_trapez[:,2], label="Trapez")
    axs[2].plot(T_RRT,cartesian_RRT[:,2], label="RRT")
    axs[2].set_ylabel("Z Axis [m]")
    axs[2].set_xlabel("Time [s]")
    axs[2].grid(True)
    axs[2].legend()

    plt.savefig("./media/directkinematics.pdf", format='pdf')

    # Plot  velocity an acceleration
    
    size_trapez = np.shape(trapez_tra)[0]
    size_RRT = np.shape(RRT_tra)[0]

    size_trapez_vel = size_trapez -1
    size_RRT_vel = size_RRT -1

    vel_trapez = np.ones((size_trapez_vel,6))
    vel_RRT = np.ones((size_RRT_vel,6))

    size_trapez_acc = size_trapez_vel -1
    size_RRT_acc = size_RRT_vel -1
    acc_trapez = np.ones((size_trapez_acc,6))
    acc_RRT = np.ones((size_RRT_acc,6))

    # create data 
    for j in range(6):
        # velocity per joints
        vel_trapez[:,j] = np.diff(trapez_tra[:,j])
        vel_RRT[:,j] = np.diff(RRT_tra[:,j])
        #acceleration per joints
        acc_trapez[:,j] = np.diff(vel_trapez[:,j])
        acc_RRT[:,j] = np.diff(vel_RRT[:,j])

    # Velocity plot
    fig, axs = plt.subplots(3)
    fig.suptitle('Velocity profile for first joint')
    for i in range(3):
        axs[i].plot(T_tra[:size_trapez_vel],vel_trapez[:,i], label=joint_name[i] + "trapez")
        axs[i].plot(T_RRT[:size_RRT_vel], vel_RRT[:,i],   label=joint_name[i] + "RRT")
        axs[i].set_ylabel("Joint" +str(i) + "Position [rd]")
        axs[i].set_xlabel("Time [s]")
        axs[i].grid(True)
        axs[i].legend()
    plt.savefig("./media/joint_velocity1_both.pdf", format='pdf')

    fig, axs = plt.subplots(3)
    fig.suptitle('Velocity profile for second joint')
    for i in range(3):
        axs[i].plot(T_tra[:size_trapez_vel],vel_trapez[:,i+3], label=joint_name[i+3] + "trapez")
        axs[i].plot(T_RRT[:size_RRT_vel], vel_RRT[:,i+3],   label=joint_name[i+3] + "RRT")
        axs[i].set_ylabel("Joint" +str(i+3) + "Position [rd]")
        axs[i].set_xlabel("Time [s]")
        axs[i].grid(True)
        axs[i].legend()
    plt.savefig("./media/joint_velocity2_both.pdf", format='pdf')

    # Acceleration plot 
    fig, axs = plt.subplots(3)
    fig.suptitle('Acceleration profile for first joint')
    for i in range(3):
        axs[i].plot(T_tra[:size_trapez_acc],acc_trapez[:,i], label=joint_name[i] + "trapez")
        axs[i].plot(T_RRT[:size_RRT_acc], acc_RRT[:,i],   label=joint_name[i] + "RRT")
        axs[i].set_ylabel("Joint" +str(i) + "Position [rd]")
        axs[i].set_xlabel("Time [s]")
        axs[i].grid(True)
        axs[i].legend()
    plt.savefig("./media/joint_acc1_both.pdf", format='pdf')

    fig, axs = plt.subplots(3)
    fig.suptitle('Acceleration profile for second joint')
    for i in range(3):
        axs[i].plot(T_tra[:size_trapez_acc],acc_trapez[:,i+3], label=joint_name[i+3] + "trapez")
        axs[i].plot(T_RRT[:size_RRT_acc], acc_RRT[:,i+3],   label=joint_name[i+3] + "RRT")
        axs[i].set_ylabel("Joint" +str(i+3) + "Position [rd]")
        axs[i].set_xlabel("Time [s]")
        axs[i].grid(True)
        axs[i].legend()
    plt.savefig("./media/joint_acc2_both.pdf", format='pdf')
    
    fig, axs1 = plt.subplots(3)
    fig.suptitle('Trajectory joint profiles for the box of')

    for i in range(3):
        axs1[i].plot(T_tra,trapez_tra[:,i], label=joint_name[i] + "trapez")
        axs1[i].plot(T_RRT,RRT_tra[:,i], label=joint_name[i] + "RRT")
        axs1[i].set_ylabel("Joint" +str(i) + "Position [rd]")
        axs1[i].set_xlabel("Time [s]")
        axs1[i].grid(True)
        axs1[i].legend()
    plt.savefig("./media/joint_trajectory1_both.pdf", format='pdf')

    fig, axs2 = plt.subplots(3)
    fig.suptitle('Trajectory joint profiles')
    for j in range(3):
        axs2[j].plot(T_tra,trapez_tra[:,j+3], label=joint_name[j+3]+ "trapez")
        axs2[j].plot(T_RRT,RRT_tra[:,j+3], label=joint_name[j+3]+ "RRT")
        axs2[j].set_ylabel("Joint" +str(j+3) + "Position [rd]")
        axs2[j].set_xlabel("Time [s]")
        axs2[j].grid(True)
        axs2[j].legend()
    plt.savefig("./media/joint_trajectory2_both.pdf", format='pdf')

