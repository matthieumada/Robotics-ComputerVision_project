ROBOTICS PROJECT - PICK & PLACE IMPLEMENTATION
==============================================

This project implements a Pick and Place task using a UR5e robot in a MuJoCo 
simulation. It includes trajectory generation using both a Trapezoidal 
Point-to-Point interpolator and a pose estimation


PROJECT STRUCTURE
-----------------
- Robotics-/: 
  Main project directory.

- exercises/: 
  Contains the core implementation logic and functions for the trajectory.

- media/: 
  Destination folder for generated figures and plots.

- scene_final.xml: 
  The MuJoCo scene file.
  Note: This is a modified version of the original "scene_obstacle.xml" 
  provided by Wilbert Peter Empleo (Exercise 6). Modifications include adjusted 
  positions for the cylinder and box, and resized drop zone to match the 
  project requirements. The obstacles remain unchanged.

- Vision project/Code/:
  Contains the core implementation logic and functions for pose estimation.
  

HOW TO RUN
----------

1. Launch the Simulation (Robot Arm)
To start the robot simulation and execute the pick-and-place task:

  a. Navigate to the project folder:
     cd Robotics-

  b. Run the script:
     python main.py

  d. Interaction:
     - The MuJoCo viewer will open and a window witha scene pointcloud  will open. Close the window pointcloud.
     - In the terminal, you will see the loading of RANSAC and 2 ICP. (won't take more than 40s)
     - A new window with the pose estimated will show up. Close it and the the robotic arm will operate


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

All the function are working except that on some random orientation of the duck the robotics won't be able to grasp it. 
The reason is either the timing ( duck moves or falls) or wrong pose estimatin).
On the code you can find the orientation of the different grasping I implemented but.  
