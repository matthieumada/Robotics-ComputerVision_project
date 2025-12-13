#!/usr/bin/env python3

import os

import open3d as o3d
import numpy as np
import copy


from do_pe import do_pose_estimation
import helpers
import settings

scene_id = settings.indexes[0]
noise_level = settings.noise_levels[0]

def main():
    print(os.getcwd())

    scene_pointcloud_file_name = settings.input_folder + f'point_cloud_{scene_id:04}.pcd'
    scene_pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_name)
    
    scene_pointcloud_noisy = helpers.add_noise(scene_pointcloud, 0, noise_level)
    
    object_mesh = o3d.io.read_triangle_mesh(settings.input_folder + "duck.stl")
    object_pointcloud = object_mesh.sample_points_poisson_disk(10000)
    
    # before my process
    o3d.visualization.draw_geometries([object_pointcloud, scene_pointcloud_noisy], window_name='Pre alignment')
    
    # my function 
    estimated_pose = do_pose_estimation(scene_pointcloud_noisy, object_pointcloud)
    print("Final pose")
    print (estimated_pose)

    # one file .txt has the real postion that is why their is one picture woth perfect alignemen 
    ground_truth = np.loadtxt(settings.input_folder + f"gt_{scene_id:04}.txt")
    print("Ground truth")
    print(ground_truth)

    # Error between the solution and mine 
    print("Error")
    error_angle, error_pos = helpers.computeError(ground_truth,estimated_pose)
    print("Position error=", error_pos, "Angle Error=", error_angle)
 
    object_pointcloud.colors = o3d.utility.Vector3dVector(np.zeros_like(object_pointcloud.points) + [0,255,0])
    # display the results 
    o3d.visualization.draw_geometries([copy.deepcopy(object_pointcloud).transform(estimated_pose), scene_pointcloud_noisy], window_name='Final alignment')

    o3d.visualization.draw_geometries([copy.deepcopy(object_pointcloud).transform(ground_truth), scene_pointcloud_noisy], window_name='Perfect alignment')
    if error_angle > 5 or  error_pos>5:
        print("Failure boy change something for position index",scene_id)
    else:
        print("Success type shit")
if __name__ == "__main__":
    main()
