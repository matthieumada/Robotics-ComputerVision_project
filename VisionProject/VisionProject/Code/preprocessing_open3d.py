import open3d as o3d
import numpy as np
from tqdm import tqdm

# This function just displays the effect of one of the functions visually, feel free to ignore or remove it.
def display_removal(preserved_points, removed_points):
    removed_points.paint_uniform_color([1, 0, 0])        # Show removed points in red
    preserved_points.paint_uniform_color([0.8, 0.8, 0.8])# Show preserved points in gray
    o3d.visualization.draw_geometries([removed_points, preserved_points])

def voxel_grid(input_cloud): # voxel size >1 remove point and voxel size<1 keep points) 
    voxel_down_cloud = input_cloud.voxel_down_sample(voxel_size=0.01) 

    return voxel_down_cloud

def outlier_removal(input_cloud):
    cl, ind = input_cloud.remove_statistical_outlier(nb_neighbors=100, std_ratio=1.0)
    #display_removal(input_cloud.select_by_index(ind), input_cloud.select_by_index(ind, invert=True))
    return input_cloud.select_by_index(ind)

def spatial_filter(input_cloud):
    pts = np.asarray(input_cloud.points)
    min = pts.min(axis=0)
    max = pts.max(axis=0)
    print("min:",min)
    print("max:",max)
    passthrough = input_cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.70, -0.3, 0.9),
                                                                       max_bound=(0.70, 0.5, 1.4)))
    #display_removal(passthrough, input_cloud)
    return passthrough

def main():
    # Load pointcloud (unfiltered)
    cloud = o3d.io.read_point_cloud('cloud.pcd')

    # Show
    o3d.visualization.draw_geometries([cloud], window_name = 'Pointcloud before filtering')

    #print("PointCloud before filtering: {} data points".format(cloud.points.shape[0]))
    print("PointCloud before filtering: {} data points".format(len(cloud.points)))

    cloud_filtered = voxel_grid(cloud)
    cloud_filtered = outlier_removal(cloud_filtered)
    cloud_filtered = spatial_filter(cloud_filtered)

    #print("PointCloud after filtering: {} data points".format(cloud_filtered.points.shape[0]))
    print("PointCloud after filtering: {} data points".format(len(cloud_filtered.points)))

    o3d.io.write_point_cloud("cloud_filtered_py.pcd", cloud_filtered)

    # Show
    o3d.visualization.draw_geometries([cloud_filtered], window_name = 'Pointcloud after filtering')

if __name__ == "__main__":
    main()