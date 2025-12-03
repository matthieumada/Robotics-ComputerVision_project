import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *
from exercises.display_trajectory import display
PI = np.pi

""" 
Build a Point to point Inteprpolator with a trapezoidal velocity
As adviced, i will select 7 points to inteprolate to fulfill the pick and place task 

The whole template is from Wilbert with the asset folder, main.py, robot.py camp.py)
All the function used in exercise folder are either from mine or from Wilbert Peter Empleo 
Of course, I added some change to some  Wilbert's function 
"""

def program(d,m):
    robot = UR5robot(data=d, model=m)
    # object slection 
    #name_obj = input("Enter the object to manipulate between cylinder, box, t_block:")
    # print(type(name_obj))
    # generate robot
    robot = UR5robot(data=d, model=m)
    robot.gripper_value = 0 # gripper

    start_q = robot.get_current_q()
    print("start_q=",start_q)
    start_frame = robot.get_current_tcp() *sm.SE3.Rx(-PI) 
    print("TCP", start_frame)

   
    return robot.queue
       