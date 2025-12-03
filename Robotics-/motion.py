import numpy as np
PI = np.pi
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *
# import class to validate the trajectory while avoiding obstacles 
from exercises.exercise_6_sol import StateValidator


