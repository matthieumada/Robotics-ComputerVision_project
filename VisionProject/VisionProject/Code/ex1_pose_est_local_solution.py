import open3d as o3d
import numpy as np
from tqdm import tqdm


"""
Code from Erik Diniz Costa Lopes Lindby solution with some adaptation of the local pose estimation in exercise 7 
"""
def load_pointclouds():
    obj = o3d.io.read_point_cloud('./datasets/gnome_artificial/object-local.pcd')
    scn = o3d.io.read_point_cloud('./datasets/gnome_artificial/scene.pcd')
    return obj, scn

def show_pointclouds(obj, scn, window_name):
    o3d.visualization.draw_geometries([obj, scn], window_name=window_name)
    return

def create_kdtree(scn):
    tree = o3d.geometry.KDTreeFlann(scn)
    return tree

def set_ICP_parameters():
    it = 50
    thressq = 0.01**2
    return it, thressq

def find_closest_points(obj_aligned, tree, thressq):
    # 1) Find closest points
    corr = o3d.utility.Vector2iVector()
    for j in range(len(obj_aligned.points)):
        k, idx, dist = tree.search_knn_vector_3d(obj_aligned.points[j], 1)
        
        # Apply distance threshold to correspondences
        if dist[0] < thressq:
            corr.append((j, idx[0]))
    return corr

def estimate_transformation(obj, scn, corr):
    # 2) Estimate transformation
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    T = est.compute_transformation(obj, scn, corr)
    return T

def apply_pose(obj, T):
    # 3) Apply pose
    obj.transform(T)
    return obj

def update_result_pose_ICP(pose, T):
    # 4) Update result
    pose = T if pose is None else T @ pose
    return pose

def main():
    # Load object and scene point clouds
    obj, scn = load_pointclouds()
    obj.colors = o3d.utility.Vector3dVector(np.zeros_like(obj.points) + [255,0,0])

    # Show starting object and scene
    show_pointclouds(obj, scn, window_name='Before local alignment')

    # Create a k-d tree for scene
    tree = create_kdtree(scn)

    # Set ICP parameters
    it, thressq = set_ICP_parameters()

    # Start ICP
    pose = None
    obj_aligned = o3d.geometry.PointCloud(obj)
    for i in tqdm(range(it), desc='ICP'):
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
    show_pointclouds(obj, scn, window_name='After local alignment')

if __name__ == '__main__':
    main()