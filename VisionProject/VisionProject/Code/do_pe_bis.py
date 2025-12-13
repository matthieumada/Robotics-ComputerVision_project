import open3d as o3d
import copy
import numpy as np
from tqdm import tqdm
import random
import spatialmath as sm 



# Function from exercice 5 and 6 of Erik and comleted by me
from preprocessing_open3d import spatial_filter, outlier_removal, voxel_grid # exercice 5
from uncertainty import  add_noise # exercice 6
from Code.utility_ICP import ICP, RANSAC # exercice 7: Note that the main of the function ex3 doesn't work yet
from ex1_pose_est_local_solution import apply_pose

def do_pose_estimation(scene_pointcloud, object_pointcloud):
    print("YOU NEED TO IMPLEMENT THIS!")
    print("Okay, I will handle this: \n ---> Filtering ")

    # Size of scene point cloud 
    pts_scene = np.asarray(scene_pointcloud.points)
    print("Scene Point cloud --> Min:", pts_scene.min(axis=0), "Max:", pts_scene.max(axis=0))
    # Size of object point cloud 
    pts_object = np.asarray(object_pointcloud.points)
    print("Object Point cloud --> Min:", pts_object.min(axis=0), "Max:", pts_object.max(axis=0))
    
    # sampling 
    # remove the back feet of the table useless data because the duck is only on the table
    T = sm.SE3.Rx(np.pi/8)
    scene_pointcloud = apply_pose(scene_pointcloud, T)
    #o3d.visualization.draw_geometries([object_pointcloud, scene_pointcloud], window_name = 'before rotation')
    reussie = o3d.io.write_point_cloud("./rotation_trial.pcd",scene_pointcloud)
    scene_filtered = spatial_filter(scene_pointcloud)
    print("Spatial -> nombre de points:", len(scene_pointcloud.points),"--->", len(scene_filtered.points))
    print("scene filtering:", len(scene_pointcloud.points),"Spatial--->", len(scene_filtered.points))
    # remove outliers as the name 
    scene_filtered = outlier_removal(scene_filtered)

    # downsampling  to accelerate the process (less point) ans assume the object doesn't have outliers
    scn_filtered = voxel_grid(scene_filtered, size=0.007)
    print("scene filtering with outliers:", len(scene_filtered.points),"Voxel-->", len(scn_filtered.points))

    obj_filtered = voxel_grid(object_pointcloud, size=0.01)
    print("object filtering:", len(object_pointcloud.points),"Voxel--->", len(obj_filtered.points))
    o3d.visualization.draw_geometries([obj_filtered, scn_filtered], window_name = 'Pointcloud after filtering')

    print("Now that we filter the objetc and the scene time for --> Pose Estimation")
    # # use global pose estimation to an approximativ position 
    # obj, scn, pose_Ransac = RANSAC(obj=obj_filtered, scn=scn_filtered, it=2000, thressq=0.005**2)
    # #o3d.visualization.draw_geometries([obj, scn], window_name = 'Pointcloud after global pose estimation')
    # print("Transformation from RANSAC:", pose_Ransac )

    # # resampling for IC to have less point and ensure convergence
    # print("Second sampling of object", len(obj.points))
    # print("Second sampling of oscene", len(scn.points))

    # # use local pose estimation to increase accuracy 
    # obj, scn, pose_ICP1  = ICP(obj=obj, scn=scn, it=100, thressq=0.016)
    # print("Transformation from ICP:", pose_ICP1 )
    # # ICP for local pose estimation
    # #o3d.visualization.draw_geometries([obj, scn], window_name = 'Pointcloud after local pose estimation')

    # obj = voxel_grid(obj, size=0.007)
    # #scn = voxel_grid(scn, size=0.007)
    # print("Second voxel: object size=", len(obj.points), "scene size=", len(scn.points))
    
    # # resampling for IC to have less point and ensure convergence
    # obj, scn, pose_ICP2  = ICP(obj=obj, scn=scn, it=190, thressq=0.001)
    # # pose_ICP = np.identity(4)
    # return pose_ICP2 @ pose_ICP1 @ pose_Ransac
    return np.identity(4)