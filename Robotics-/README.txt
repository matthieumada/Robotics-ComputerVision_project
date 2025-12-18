ROBOTICS PROJECT - PICK & PLACE IMPLEMENTATION
==============================================

This project implements a Pick and Place task using a UR5e robot in a MuJoCo 
simulation. It includes trajectory generation using both a Trapezoidal 
Point-to-Point interpolator and the RRT path planner.


PROJECT STRUCTURE
-----------------
- Robotics-/: 
  Main project directory.

- exercises/: 
  Contains the core implementation logic and functions.

- media/: 
  Destination folder for generated figures and plots.

- scene_final.xml: 
  The MuJoCo scene file.
  Note: This is a modified version of the original "scene_obstacle.xml" 
  provided by Wilbert Peter Empleo (Exercise 6). Modifications include adjusted 
  positions for the cylinder and box, and resized drop zone to match the 
  project requirements. The obstacles remain unchanged.


HOW TO RUN
----------

1. Launch the Simulation (Robot Arm)
To start the robot simulation and execute the pick-and-place task:

  a. Navigate to the project folder:
     cd Robotics-

  b. Open "main.py" and ensure the desired method (Trapezoidal or RRT) 
     is uncommented if applicable.

  c. Run the script:
     python main.py

  d. Interaction:
     - The MuJoCo viewer will open.
     - In the terminal, you will be prompted to enter the name of the object 
       you want to manipulate (e.g., box, cylinder, etc.).
     - Type the name and press Enter to start the robot.


2. Generate Analysis Figures
To evaluate the trajectories and generate plots (Position, Velocity, 
Acceleration, Direct Kinematics):

  a. Run the comparison script:
     python compare_trajectory.py

  b. The resulting figures will be saved automatically inside the "media/" folder.


IMPLEMENTATION DETAILS
----------------------
- Core Logic: 
  The specific functions for trajectory planning and control are located 
  in the "exercises/" folder.

- Base Template: 
  The project structure (including asset/, main.py, robot.py, cam.py) is 
  based on the template provided by Wilbert Peter Empleo.

- Modifications: 
  Custom functions were added, and existing functions from the template were 
  modified to suit the specific needs of this assignment.


CREDITS
-------
- Base framework and initial scene assets: Wilbert Peter Empleo
- Modifications and logic implementation: Matthieu DELIN

All the function are working except the cylinder in RRT get stuck in the gripper when he wants to drop it. 
