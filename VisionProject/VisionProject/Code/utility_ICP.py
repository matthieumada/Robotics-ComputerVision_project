import open3d as o3d
import numpy as np
from tqdm import tqdm
import random

""" 
ICP set of function  useful to use it 
""" 

def update_result_pose_ICP(pose, T):
    # 4) Update result
    pose = T if pose is None else T @ pose
    return pose


def apply_pose(obj, T):
    # 3) Apply pose
    obj.transform(T)
    return obj 

def create_kdtree(scn):
    tree = o3d.geometry.KDTreeFlann(scn)
    return tree

def estimate_transformation(obj, scn, corr):
    # 2) Estimate transformation
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    T = est.compute_transformation(obj, scn, corr)
    return T

def find_closest_points(obj_aligned, tree, thressq):
    # 1) Find closest points
    corr = o3d.utility.Vector2iVector()
    for j in range(len(obj_aligned.points)):
        k, idx, dist = tree.search_knn_vector_3d(obj_aligned.points[j], 1)
        
        # Apply distance threshold to correspondences
        if dist[0] < thressq:
            corr.append((j, idx[0]))
    return corr

def ICP(obj, scn, it, thressq):
    print("Starting local estimation with ICP")
    """
    Implement the code from ex1_pose_est_local_solution.py here.
    """
    # Purpose align an object point cloud to a scene point cloud using ICP (Iterative Closest Point) algorithm.
    # Load object and scene point clouds
    # obj, scn = load_pointclouds()
    obj.colors = o3d.utility.Vector3dVector(np.zeros_like(obj.points) + [255,0,0])

    # Show starting object and scene
    #show_pointclouds(obj, scn, window_name='Before local alignment')
    #o3d.io.write_point_cloud("./result/Before_local_alignment_object.pcd", [obj, scn])
    # Create a k-d tree for scene
    tree = create_kdtree(scn)
    print("KD-Tree created",tree)
    # Set ICP parameters
    #it, thressq = set_ICP_parameters()

    # Start ICP
    pose = None
    obj_aligned = o3d.geometry.PointCloud(obj)
    for i in tqdm(range(it), desc='ICP'): # tqdm for progress bar 
        # 1) Find closest points
        corr = find_closest_points(obj_aligned, tree, thressq)
            
        # 2) Estimate transformation
        T = estimate_transformation(obj_aligned, scn, corr)
        
        # 3) Apply pose
        obj_aligned = apply_pose(obj_aligned, T)
        
        # 4) Update result
        pose = update_result_pose_ICP(pose, T)

    # Print pose
    print('Got the following pose:')
    print(pose)

    # Apply pose to the original object
    obj = apply_pose(obj, pose)

    # Show result object and scene
    #show_pointclouds(obj, scn, window_name='After local alignment')
    #o3d.io.write_point_cloud("./result/ICP_try.pcd", [obj, scn])
    return obj, scn, pose


