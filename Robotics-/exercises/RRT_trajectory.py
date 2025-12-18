import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *
from exercises.display_trajectory import display
# from wilbert exercise importing class 
from exercises.exercise_6_sol import StateValidator
from exercises.exercise_7 import path_prune_simple # to filter the point put increase the likelihood of failure 
PI = np.pi

""" 

Build a Rapidly-Exploring random Three. The function are from the exercice 6. except program and drop_object especially one attempt was to filter 
the point but it increases the likelihood to hit an obstacles. 
"""
def plan(d, m, start_q, goal_q):
    num_joint = 6
    
    space = ob.RealVectorStateSpace(num_joint) # Create a joint-space vector instead of a 2D space as in the example
    # Create joint bounds
    bounds = ob.RealVectorBounds(num_joint)
    bounds.setLow(-3.14)
    bounds.setHigh(3.14)
    space.setBounds(bounds)
    
    # Create SimpleSetup
    ss = og.SimpleSetup(space)
    validator = StateValidator(d, m, num_joint)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validator))
    
    # Set start and goal states
    start = ob.State(space)
    goal = ob.State(space)
    
    # Set specific joint values for start and goal
    for i in range(num_joint):
        start[i] = start_q[i] # initial joint angles
        goal[i] = goal_q[i]  # goal joint angles (~90 degrees)
    
    ss.setStartAndGoalStates(start, goal)
    
    # RRT planner setting
    # fatser to explore huge space and works with a few of samples
    # path is jerk and not optimize and doesn't fit to multiple request
    ss.setPlanner(og.RRT(ss.getSpaceInformation()))
    
    # Solve the problem
    solved = ss.solve(10.0)
    
    # no interpolation for smoothing the trajetory
    if solved:
        # Get the solution path
        path = ss.getSolutionPath()
        print("Found a Solution!")
        # Print basic information about the path
        print(f"Path length: {path.length()}")
        print(f"Number of states: {path.getStateCount()}")

        solution_trajectory = []
        for i in range(path.getStateCount()):
            state = path.getState(i)
            q_pose = [state[i] for i in range(space.getDimension())]
            print(f"State {i}: {q_pose}")
            solution_trajectory.append(np.array(q_pose))
        return solution_trajectory


def drop_object(m, d, name_obj, robot, opposite_start_q):
    drop_frame = get_mjobj_frame(model=m, data=d, obj_name="zone_drop")* sm.SE3.Rx(-PI) * sm.SE3.Tz(-0.05)
    goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame, q0= opposite_start_q)[0]
    return drop_frame, goal_q


def program(d,m):
    robot = UR5robot(data=d, model=m)
    # object selection 
    name_obj = input("Enter the object to manipulate between: cylinder, box, t_block:")
    # # Case when we write wrong the name 
    if name_obj not in [ "cylinder", "box", "t_block"]:
        raise Exception("Please check the name of object selected") 
    
    robot.gripper_value = 0 # gripper

    start_q = robot.get_current_q()
    print("start_q=",start_q)
    start_frame = robot.get_current_tcp() *sm.SE3.Rx(-PI) 
    print("TCP", start_frame)
    if name_obj == "box":
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name="box") * sm.SE3.Rx(-PI)
        
        close_obj_frame = obj_frame * sm.SE3.Tz(-0.2) 
         # go closer 20 cm above to avoid the cylinder
        goal_q = robot.robot_ur5.ik_LM(Tep=close_obj_frame, q0= start_q)[0]
        trajectory = plan(d=d, m=m, start_q=start_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=600)

        # get to grasp it
        real_q = goal_q 
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0= real_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=600)
        

        # grasp it 
        real_q = goal_q
        robot.set_gripper(value=255,t=100)  

        # go closer 20 cm above to avoid the cylinder
        goal_q = robot.robot_ur5.ik_LM(Tep=close_obj_frame, q0= real_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)

        #   go to the opposite start_q
        real_q = goal_q
        opposite_start_q = start_q + [-2.8300000000000005, 0, 0, 0, 0, 0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=opposite_start_q)
        robot.move_j_via(points=trajectory, t=800)

        drop_frame, goal_q = drop_object(m=m, d=d, robot=robot, name_obj=name_obj, opposite_start_q=opposite_start_q)
        trajectory = plan(d=d, m=m, start_q=opposite_start_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)
        print("box selected")
        

    elif name_obj == "t_block":
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name="t_block") * sm.SE3.Rx(-PI) 
        close_frame = obj_frame * sm.SE3.Tz(-0.01) * sm.SE3.Rz(PI/2)
        drop_frame = get_mjobj_frame(model=m, data=d, obj_name="zone_drop") *sm.SE3.Rx(-PI) * sm.SE3.Tz(-0.05)

        goal_q = robot.robot_ur5.ik_LM(Tep=close_frame * sm.SE3.Ty(-0.05), q0= start_q)[0]
        trajectory = plan(d=d, m=m, start_q=start_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=200)
        print("robot.queue:",robot.queue)

        # grasp it
        robot.set_gripper(value=255, t=100) 
        real_q = goal_q

        # return to start 
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=start_q)
        robot.move_j_via(points=trajectory, t=800)
        real_q = start_q
        oal_q= start_q + [-2.8300000000000005, 0, 0, 0, 0, 0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)

        real_q = goal_q
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame, q0=real_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)
        print("T_block selected")

    else:
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name="cylinder") * sm.SE3.Rx(-PI) 
        close_frame = obj_frame * sm.SE3.Tz(-0.01) * sm.SE3.Rz(PI/2)
        drop_frame = get_mjobj_frame(model=m, data=d, obj_name="zone_drop") *sm.SE3.Rx(-PI) * sm.SE3.Tz(-0.05)
        obj_frame = start_frame * sm.SE3.Ry(-PI/2)

        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=start_q)[0]
        trajectory = plan(d=d, m=m, start_q=start_q, goal_q = goal_q)
        robot.move_j_via(points=trajectory, t=800)

        real_q = goal_q
        obj_frame = obj_frame * sm.SE3.Rz(-PI/2)
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame , q0=real_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q = goal_q)
        robot.move_j_via(points=trajectory, t=800)

        real_q = goal_q
        obj_frame = obj_frame * sm.SE3.Tz(-0.12) * sm.SE3.Tx(-0.10)
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=goal_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q = goal_q)
        robot.move_j_via(points=trajectory, t=800)

        real_q = goal_q
        # now around 10 cm above the target we can go down to grasp it
        goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame * sm.SE3.Ty(-0.225), q0=goal_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q = goal_q)
        robot.move_j_via(points=trajectory, t=800)

        # grasp it
        robot.set_gripper(value=250, t=100) 
        real_q = goal_q

        # return to start 
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=start_q)
        robot.move_j_via(points=trajectory, t=800)
        real_q = start_q
        goal_q= start_q + [-2.8300000000000005, 0, 0, 0, 0, 0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)
        real_q = goal_q
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame * sm.SE3.Tz(-0.03), q0=real_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)

         # cylinder stuck in the gripper 
        robot.set_gripper(value=120, t=300)
        robot.set_gripper(value=40, t=300)
        
        real_q = goal_q
        goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame * sm.SE3.Tz(-0.035), q0=real_q)[0]
        trajectory = plan(d=d, m=m, start_q=real_q, goal_q=goal_q)
        robot.move_j_via(points=trajectory, t=800)
        robot.set_gripper(value=0, t=300)
        robot.set_gripper(value=0, t=300)
        
        print("Cylinder selected")
       
    #drop object
    robot.set_gripper(value=0, t=300)
    # return to start
    trajectory = plan(d=d, m=m, start_q=goal_q, goal_q=start_q + [-2.8300000000000005, 0, 0, 0, 0, 0])
    robot.move_j_via(points=trajectory, t=200)

    real_q = start_q + [-2.8300000000000005, 0, 0, 0, 0, 0]
    trajectory = plan(d=d, m=m, start_q=real_q, goal_q=start_q )
    robot.move_j_via(points=trajectory, t=200)


    data = [q_pose for q_pose, _ in robot.queue] # one joint value to plot # data is a list of numpy arrays
    data = np.array(data) 
    display(data=data, name_obj=name_obj, method="RRT")
    print(robot.queue)
    return robot.queue
       