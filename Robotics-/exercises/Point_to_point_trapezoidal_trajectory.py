
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random
from ompl import base as ob
from ompl import geometric as og
from robot import *
# import the function to display trajectory 
from exercises.display_trajectory import display
# constant 
PI = np.pi

""" 
Build a Point to point Inteprpolator with a trapezoidal velocity
By using the advice, I will select 8 points to inteporlate to fulfill the pick and place task. After come back to the initial position. 

The whole template is from Wilbert with the asset folder, main.py, robot.py camp.py)
All the Python function used in exercise folder are either from mine or from Wilbert Peter Empleo 
Of course, I added some change to some  Wilbert's function 

The scene_final.xml is a modified verison of scene_obstacle.xml given by Wilbert for the exercise 6. I only changed the size 
of the drip zone and the position of the cylinder and the box to fit with picture given in the instruction. I could have change the size of the obstacles 
but I prefered to keep them as they are.
"""

def trapezoidal_trajectory(robot, via_q):
    # Trapezoidal interpolation with control of the speed by the number of steps 
    if robot.gripper_value > 0:
        t = 450
    else:
        t = 300
    print("Interpolate trajectory")
    via_q = np.array((via_q))
    L = np.shape(via_q)[0]
    for joint in range(1, L):
        traj = rtb.mtraj(tfunc=rtb.trapezoidal, q0=via_q[joint-1], qf=via_q[joint], t=t) # t=number of steps
        for step in traj.s: # .s to get position information:
            robot.queue.append((step, robot.gripper_value))
    return 

def pick_object(m, d, robot, name_obj, start_q):
    # starting joint 
    q_order = [start_q]
    print(name_obj)

    if name_obj =='box':
        print( "Picking box")
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name="box") * sm.SE3.Rx(-PI) 
        print("obj_frame=",obj_frame)

        close_frame = obj_frame * sm.SE3.Tz(-0.2)  # 10 cm above
        goal_q = robot.robot_ur5.ik_LM(Tep=close_frame, q0=start_q)[0]
        q_order.append(goal_q)
    
        # move closer to catch the box
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=goal_q)[0]
        q_order.append(goal_q)

    elif name_obj =='t_block':
        print( "Picking t_block")
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name="t_block") * sm.SE3.Rx(-PI) 
        print("obj_frame=",obj_frame)

        close_frame = obj_frame * sm.SE3.Tz(-0.2)  # 10 cm above
        goal_q = robot.robot_ur5.ik_LM(Tep=close_frame, q0=start_q)[0]
        q_order.append(goal_q)
    
        # move closer to catch the box
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame* sm.SE3.Rz(-PI/2), q0=goal_q)[0]
        q_order.append(goal_q)
    
    elif name_obj == "cylinder_top":
        # version grasping on the top 
        print( "Picking cylinder")
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name="cylinder") * sm.SE3.Rx(-PI) 
        print("obj_frame=",obj_frame)

        close_frame = obj_frame * sm.SE3.Tz(-0.3)   # 30 cm above
        goal_q = robot.robot_ur5.ik_LM(Tep=close_frame, q0=start_q)[0]
        q_order.append(goal_q)

        close_frame = obj_frame * sm.SE3.Tz(-0.2)   # 20 cm above
        goal_q = robot.robot_ur5.ik_LM(Tep=close_frame, q0=start_q)[0]
        q_order.append(goal_q)
    
        # move closer to catch the box
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame *sm.SE3.Tz(-0.05) , q0=goal_q)[0]
        q_order.append(goal_q)
    
    else:
        # version grasping on the side
        start_frame = robot.get_current_tcp() *sm.SE3.Rx(-PI)
        obj_frame = start_frame * sm.SE3.Ry(-PI/2)
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=start_q)[0]
        q_order.append(goal_q)

        obj_frame = obj_frame * sm.SE3.Rz(-PI/2)
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame , q0=goal_q)[0]
        q_order.append(goal_q)

        obj_frame = obj_frame * sm.SE3.Tz(-0.12) * sm.SE3.Tx(-0.10)
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=goal_q)[0]
        q_order.append(goal_q)

        # now around 10 cm above the target we can go down to grasp it
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame * sm.SE3.Ty(-0.225), q0=goal_q)[0]
        q_order.append(goal_q)
    
    # create trajectory and grasp it
    trapezoidal_trajectory(robot, q_order)
    robot.set_gripper(200) 
    print("object picked")
    return obj_frame, goal_q

def drop_object(robot, name_obj, q_order, drop_frame):
    # drop object
    goal_q = q_order[-1]
    if name_obj == "cylinder_top":
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame *  sm.SE3.Tz(-0.125), q0=goal_q)[0] # add height for cylinder
        q_order.append(goal_q)

    
    elif name_obj == "cylinder_side":
        print("chose")
        # drop_frame = drop_frame * sm.SE3.Rx(PI/2)
        # print(robot.robot_ur5.ik_LM(Tep=drop_frame, q0=goal_q))
        # goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame, q0=goal_q)[0]
        # q_order.append(goal_q)
        
        drop_frame = drop_frame *sm.SE3.Tz(-0.3) * sm.SE3.Ry(-PI/2) 
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame, q0=goal_q)[0] # add height for cylinder
        q_order.append(goal_q)

        drop_frame = drop_frame * sm.SE3.Rx(PI/2) 
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame, q0=goal_q)[0] # add height for cylinder
        q_order.append(goal_q)

        # drop_frame = drop_frame * sm.SE3.Tx(0.2)
        # goal_q = robot.robot_ur5.ik_LM(Tep= drop_frame, )

        # goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame *  sm.SE3.Tz(-0.225), q0=goal_q)[0] # add height for cylinder
        # q_order.append(goal_q)
        
    else:
        print("not chose")
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame, q0=goal_q)[0]
        q_order.append(goal_q)


    trapezoidal_trajectory(robot, q_order)
    robot.set_gripper(0) # open gripper to drop
    q_order = [goal_q]
    return q_order, goal_q



def program(d, m):
    # object slection 
    name_obj = input("Enter the object to manipulate between: cylinder_side, cylinder_top, box, t_block:")

    # Case when we write wrong the name 
    if name_obj not in ["cylinder_side", "cylinder_top", "box", "t_block"]:
        raise Exception("Please check the name of object selected") 

    # print(type(name_obj))
    # generate robot
    robot = UR5robot(data=d, model=m)
    robot.gripper_value = 0 # gripper

    start_q = robot.get_current_q()
    print("start_q=",start_q)
    start_frame = robot.get_current_tcp() *sm.SE3.Rx(-PI) 
    print("TCP", start_frame)
    
    #Define grasping frames for object: box
    obj_frame, goal_q = pick_object(m, d, robot, name_obj, start_q)
    q_order = [goal_q]

    # now move up and go to the start
    close_frame = obj_frame * sm.SE3.Tz(-0.2) # 10 cm above
    goal_q = robot.robot_ur5.ik_LM(Tep=close_frame, q0=goal_q)[0]
    q_order.append(goal_q)

    # Using the control command of Mujoco, I found this values just to avoid the obstcales safely
    # In fact, only 1 joint move the other are constant so it is not so hard to code 
    
    # strech out the elbow 
    goal_q = [-0.664, -1.15, 0.0318, -1.75, -1.57, -2.23]
    q_order.append(goal_q)

    #  rotate to align 
    goal_q = [-0.664, -1.45, 0.0318, -3.46, -1.57, -2.23]
    q_order.append(goal_q)

    # pass between the two obstacles 
    goal_q = [-2.83, -1.45, 0.0318, -3.46, -1.57, -2.23]
    q_order.append(goal_q)

    # # rotate the wrist to prepare for drop by suing the start q position 
    goal_q = start_q + [-2.8300000000000005, 0, 0, 0, 0, 0]
    q_order.append(goal_q)

    # Now I can use the drop_zone position to define the drop frame
    drop_frame = get_mjobj_frame(model=m, data=d, obj_name="zone_drop") *sm. SE3.Rx(-PI) * sm.SE3.Tz(-0.05)
    close_drop_frame = drop_frame * sm.SE3.Tz(-0.3) # 20 cm above   
    goal_q = robot.robot_ur5.ik_LM(Tep=close_drop_frame, q0=goal_q)[0]
    q_order.append(goal_q)
    q_order, goal_q = drop_object(robot=robot, name_obj=name_obj, q_order=q_order, drop_frame=drop_frame)
    
    # move up after drop
    goal_q = robot.robot_ur5.ik_LM(Tep=close_drop_frame, q0=goal_q)[0]
    q_order.append(goal_q)

    trapezoidal_trajectory(robot, q_order)
    q_order = [goal_q]

    # return to start position 
    # pass between the two obstacles 
    goal_q = [-2.83, -1.45, 0.0318, -3.46, -1.57, -2.23]
    q_order.append(goal_q)

    #  rotate to align 
    goal_q = [-0.664, -1.45, 0.0318, -3.46, -1.57, -2.23]
    q_order.append(goal_q)
    q_order.append(start_q)

    trapezoidal_trajectory(robot, q_order)
    

    # # Check if there is any singularity in the trajectory created 
    id = 0 
    for joint in robot.queue:
        jac= robot.robot_ur5.jacob0(joint[0])
        det = np.linalg.det(jac)
        rtb.jsingu(jac)
        if rtb.jsingu(jac) != None:
            print("Warning: Singularity detected during the trajectory execution")
            print("Joint position:", joint)
            print(robot.queue)
            print("id=",id)
            break
        id += 1 

 
    data = [q_pose for q_pose, _ in robot.queue] # one joint value to plot # data is a list of numpy arrays
    data = np.array(data)  # Convert list to numpy array for easier indexing
    display(data, name_obj)

    return robot.queue
       
         
