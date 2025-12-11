import open3d as o3d
import numpy as np
from tqdm import tqdm
import random

def load_pointclouds():
    """
    Load the object and scene point clouds from the dataset you want to use.
    """
    obj = o3d.io.read_point_cloud('./datasets/gnome_artificial/object-global.pcd')
    scn = o3d.io.read_point_cloud('./datasets/gnome_artificial/scene.pcd')
    return obj, scn

def show_pointclouds(obj, scn, window_name):
    o3d.visualization.draw_geometries([obj, scn], window_name=window_name)
    return

def set_RANSAC_parameters():
    it = 1000
    thressq = 0.01**2
    return it, thressq

def compute_surface_normals(obj, scn):
    obj.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(10))
    scn.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(10))
    return

def compute_shape_features(obj, scn):
    obj_features = o3d.pipelines.registration.compute_fpfh_feature(obj, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))
    scn_features = o3d.pipelines.registration.compute_fpfh_feature(scn, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))
    return obj_features, scn_features

def find_feature_matches(obj_features, scn_features):
    corr = o3d.utility.Vector2iVector()
    for j in tqdm(range(obj_features.shape[0]), desc='Correspondences'):
        fobj = obj_features[j]
        dist = np.sum((fobj - scn_features)**2, axis=-1)
        kmin = np.argmin(dist)
        corr.append((j, kmin))
    return corr

def create_kdtree(scn):
    tree = o3d.geometry.KDTreeFlann(scn)
    return tree

def apply_pose(obj, T):
    obj.transform(T)
    return obj

def sample_3_random_correspondences(corr):
    # Sample 3 random correspondences
    random_corr = o3d.utility.Vector2iVector(random.choices(corr, k=3))
    return random_corr

def estimate_transformation(obj, scn, corr):
    # Estimate transformation
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    T = est.compute_transformation(obj, scn, corr)
    return T

def validate(obj_aligned, tree, thressq):
    inliers = 0
    for j in range(len(obj_aligned.points)):
        k, idx, dist = tree.search_knn_vector_3d(obj_aligned.points[j], 1)
        if dist[0] < thressq:
            inliers += 1
    return inliers

def update_result_pose(pose_best, T, inliers, inliers_best, obj):
    if inliers > inliers_best:
        print(f'Got a new model with {inliers}/{len(obj.points)} inliers!')
        inliers_best = inliers
        pose_best = T
    else:
        pose_best = pose_best
        inliers_best = inliers_best
    return pose_best, inliers_best

def main():
    # Load object and scene point clouds
    obj, scn = load_pointclouds()
    obj.colors = o3d.utility.Vector3dVector(np.zeros_like(obj.points) + [255,0,0])

    # Show starting object and scene
    show_pointclouds(obj, scn, window_name='Before global alignment')

    # Set RANSAC parameters
    it, thressq = set_RANSAC_parameters()

    # Compute surface normals
    compute_surface_normals(obj, scn)

    # Compute shape features
    obj_features, scn_features = compute_shape_features(obj, scn)

    obj_features = np.asarray(obj_features.data).T
    scn_features = np.asarray(scn_features.data).T

    # Find feature matches
    corr = find_feature_matches(obj_features, scn_features)

    # Create a k-d tree for scene
    tree = create_kdtree(scn)

    # Start RANSAC
    random.seed(123456789)
    inliers_best = 0
    pose_best = None
    for i in tqdm(range(it), desc='RANSAC'):   
        # Sample 3 random correspondences
        corr_i = sample_3_random_correspondences(corr)
        
        # Estimate transformation
        T = estimate_transformation(obj, scn, corr_i)
        
        # Apply pose (to a copy of the object)
        obj_aligned = o3d.geometry.PointCloud(obj)
        obj_aligned = apply_pose(obj_aligned, T)
        
        # Validate
        inliers = validate(obj_aligned, tree, thressq)

        # Update result
        pose_best, inliers_best = update_result_pose(pose_best, T, inliers, inliers_best, obj_aligned)

    # Print pose
    print('Got the following pose:')
    print(pose_best)

    # Apply pose to the original object
    obj = apply_pose(obj, pose_best)

    # Show result
    show_pointclouds(obj, scn, window_name='After global alignment')
    
if __name__ == '__main__':
    main()