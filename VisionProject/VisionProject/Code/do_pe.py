import open3d as o3d
import copy
import numpy as np
from tqdm import tqdm
import random

# Function from exercice 5 and 6 of Erik and comleted by me
from preprocessing_open3d import spatial_filter, outlier_removal, voxel_grid # exercice 5
from uncertainty import  add_noise # exercice 6

def do_pose_estimation(scene_pointcloud, object_pointcloud):
    print("YOU NEED TO IMPLEMENT THIS!")
    print("Okay, I will handle this: \n ---> Filtering ")

    # Size of scene point cloud 
    pts_scene = np.asarray(scene_pointcloud.points)
    print("Scene Point cloud --> Min:", pts_scene.min(axis=0), "Max:", pts_scene.max(axis=0))
    # Size of object point cloud 
    pts_object = np.asarray(scene_pointcloud.points)
    print("Object Point cloud --> Min:", pts_object.min(axis=0), "Max:", pts_object.max(axis=0))

    # sampling 
    # remove the back feet of the table useless data because the duck is only on the table
    scene_filtered = spatial_filter(scene_pointcloud)

    # remove outliers as the name 
    scene_filtered = outlier_removal(scene_filtered)

    # downsamping  to accelerate the eprocess (less point)
    #scene_filtered = voxel_grid(scene_filtered)

    
    
    o3d.visualization.draw_geometries([scene_filtered], window_name = 'Pointcloud after filtering')


    return np.identity(4)