import mujoco as mj
from spatialmath import SE3, SO3
from spatialmath.base import trnorm
from scipy.spatial.transform import Rotation
import math
import random
import mujoco

from cam import * # camera function 
from robot import * # robot function 
from VisionProject.Code.do_pe import do_pose_estimation # computer vision function
import VisionProject.Code.settings as settings 
from exercises.Point_to_point_trapezoidal_trajectory import trapezoidal_trajectory
from exercises.display_trajectory import display
PI = np.pi

def r2q(rot):
    """
    Convert a 3x3 rotation matrix to a quaternion [w, x, y, z]
    using scipy's Rotation class
    """
    r = Rotation.from_matrix(rot)
    return r.as_quat()  # Returns [x, y, z, w]

def program(d, m):
    # Computer vision
    # _width = 640*3,
    # _height = 480*3,
    # Initialize OpenGL context
    # mj.GLContext(max_width=640, max_height=480)
    # Create renderer

    camera_name = "cam1"

    # Random duck position
    rand_x = random.uniform(0.45,0.75)
    rand_y = random.uniform(-0.55,0.55)

    z = 0.025
    d.joint('duck').qpos [0:3]= [rand_x, rand_y, z+0.01]
    rand_rot = random.randint(0,359)
    rot = SO3.Eul(225, 90, 90,unit="deg").R
    d.joint('duck').qpos[3:] = r2q(rot)

    duck_pos = d.body('duck').xpos
    duck_rot = d.body('duck').xmat.reshape(3, 3)
    duck_rot = trnorm(duck_rot)
    duck_se3 = SE3.Rt(duck_rot, duck_pos)

    mujoco.mj_step(m, d)


    cam_se3_2 = get_camera_pose_cv(m, d, camera_name=camera_name)

    gt = cam_se3_2.inv() * duck_se3

    id = 0

    with open(f"gt_{id:04}.txt", 'w') as f:
        for i in range(4):
            for j in range(4):
                f.write(f"{gt.A[i,j]} ")
            f.write("\n")
    
    renderer = mj.Renderer(m, height=480, width=640)
    get_pointcloud(m, d, renderer, f"point_cloud_{id:04}.pcd", camera_name=camera_name)
    show_pointcloud(f"point_cloud_{id:04}.pcd")

    # load scene 
    scene_pointcloud = o3d.io.read_point_cloud(f"point_cloud_{id:04}.pcd")

    # load object 
    object_mesh = o3d.io.read_triangle_mesh(settings.input_folder + "duck.stl")
    object_pointcloud = object_mesh.sample_points_poisson_disk(10000)

    # pose estimation
    pose =  do_pose_estimation(scene_pointcloud, object_pointcloud) 
    estimated_pose = cam_se3_2* pose
    print("Pose computation finished, estimate:", estimated_pose)
    estimated_pose = estimated_pose * sm.SE3.Rx(PI/2)
    print("type:",type(estimated_pose))

    # compute trajectory 
    robot = UR5robot(data=d, model=m)
    robot.gripper_value = 0 # gripper
    # current posiiton
    start_q = robot.get_current_q()
    start_frame = robot.get_current_tcp()
    R_start_frame = start_frame.R 
    print("Rotation start_frame:", R_start_frame)
    rpy = SO3(trnorm(pose[:3,:3])).rpy(unit='deg', order='xyz')
    print("x:",rpy[0], "y:", rpy[1], "z:", rpy[2] )
    print("poisiton from estimated pose", estimated_pose[:3,3])
    pose = SE3.Rt(R=R_start_frame,t= estimated_pose[:3,3]) 
    print("useful pose")
    q_order = [start_q]

    # Get closer to the duck 
    pose_above =  pose * sm.SE3.Tz(-0.1)
    goal_q = robot.robot_ur5.ik_NR(Tep=pose_above, q0=start_q)[0]
    q_order.append(goal_q)
    print("goal_q", goal_q)
    if  -145< rpy[2]< -100:
    # grasp the duck 
        #rand_rot = 0 and 335
        print("the duck is up")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(-0.019) * sm.SE3.Tz(-0.022), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250

    elif 130 < rpy[2] < 155 or -100<= rpy[2] < -90:
        print("duck move forward")
        # rand_rot = 270
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(-0.15) * sm.SE3.Tz(-0.015), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250
     
    elif  -65< rpy[2] < 0: 
        # grasp the duck 
        # rand_rot = 90
        print("the duck is upside down")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(-0.022) * sm.SE3.Tz(-0.005), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 225
    
    elif 0 < rpy[2] < 40:
         # grasp the duck 
        # rand_rot = 135
        print("duck on the head")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(0.022) * sm.SE3.Tz(-0.02), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250

    elif 90< rpy[2] <=120 :
         # grasp the duck 
         # rand_rot = 225
        print("the duck is back ")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose *  sm.SE3.Ty(-0.05) *  sm.SE3.Tz(0.02), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250

    elif -80 < rpy[2] < -65 :
        # rand_rot = 45
        print("duck 45")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose *  sm.SE3.Ty(0.002) *  sm.SE3.Tz(-0.02), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250

    # some obtain by tuning on random position 
    elif -170 < rpy[2] < -155 :
        print("oscillating duck")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose *  sm.SE3.Ty(-0.033) *  sm.SE3.Tz(-0.023), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 225
    
    elif 155 < rpy[2] < 180:
        print("duck move fully forward")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(-0.06) * sm.SE3.Tz(-0.008), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250

    elif 120 < rpy[2] < 130:
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(0.015) * sm.SE3.Tz(-0.008), q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250

    elif 50 < rpy[2] <= 80 :
        print("duck little move forward ")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(-0.015) , q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 230

    else :
        print("the duck face is down  ")
        goal_q = robot.robot_ur5.ik_NR(Tep=pose * sm.SE3.Ty(0.022) *  sm.SE3.Tz(-0.008) , q0=goal_q)[0]
        q_order.append(goal_q)
        grip = 250


    trapezoidal_trajectory(robot, q_order)
    robot.set_gripper(value=grip, t=300) 
    q_order = [goal_q]
    q_order.append(start_q)

    drop_frame = get_mjobj_frame(model=m, data=d, obj_name="zone_drop") * sm. SE3.Rx(-PI) * sm.SE3.Tz(-0.05)
    # drop_close_frame = drop_frame * sm.SE3.Tz(-0.2)
    # goal_q = robot.robot_ur5.ik_LM(Tep=drop_close_frame, q0=start_q)[0]
    # q_order.append(goal_q)

    goal_q = robot.robot_ur5.ik_LM(Tep=drop_frame* sm.SE3.Tz(-0.08), q0=start_q)[0]
    q_order.append(goal_q)
    trapezoidal_trajectory(robot, q_order)

    # duck is stuck in the gripper 
    robot.set_gripper(value=120, t=300)
    robot.set_gripper(value=0, t=300)
    
    q_order = [goal_q]

    # return to start position
    q_order.append(start_q)
    trapezoidal_trajectory(robot, q_order)
    data = [q_pose for q_pose, _ in robot.queue] # one joint value to plot # data is a list of numpy arrays
    data = np.array(data)  # Convert list to numpy array for easier indexing
    display(data, "duck", method="PIP_trapezoidal")
    return robot.queue

    















