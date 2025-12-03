import matplotlib.pyplot as plt
import numpy as np 

""" 
This file is use to print the position, velocity, acceleration and jerk from all joints for both method"
You can change the joint in the line joint_0 = np.array(data[:,4]) to select the joint you want to plot
 display
 --- Input :  data: numpy array of joint positions 
              name_obj: string of identfiy the object used in trajectory
 --- Output: None 
"""

def display(data, name_obj):

    joint_0 = data[:,0] # select the first joint 

    # Plot trajectory profile of joint 0
    fig, axs = plt.subplots(4)
    fig.suptitle('Trajectory profiles of joint 0' + name_obj)

    vel = np.diff(joint_0)
    acc = np.diff(vel)
    jerk = np.diff(acc)

    axs[0].plot(joint_0)
    axs[1].plot(vel)
    axs[2].plot(acc)
    axs[3].plot(jerk)

    axs[0].set_ylabel("Position")
    axs[1].set_ylabel("Velocity")
    axs[2].set_ylabel("Accelaration")
    axs[3].set_ylabel("Jerk")

    plt.savefig("./media/trajectory.pdf", format='pdf')

    # plot position of all joints
    fig, axs1 = plt.subplots(3)
    fig.suptitle('Trajectory joint profiles of' + name_obj)
    joint_name = ["soulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    for i in range(3):
        axs1[i].plot(data[:,i], label=joint_name[i])
        axs1[i].set_ylabel("Joint" +str(i) + "Position [rd]")
        axs1[i].set_xlabel("Time [s]")
        axs1[i].grid(True)
        axs1[i].legend()
    plt.savefig("./media/joint_trajectory1.pdf", format='pdf')

    fig, axs2 = plt.subplots(3)
    fig.suptitle('Trajectory joint profiles' + name_obj)
    for j in range(3):
        axs2[j].plot(data[:,j+3], label=joint_name[j+3])
        axs2[j].set_ylabel("Joint" +str(j+3) + "Position [rd]")
        axs2[j].grid(True)
        axs2[j].set_xlabel("Time [s]")
        axs2[j].legend()
    plt.savefig("./media/joint_trajectory2.pdf", format='pdf')
    return 

