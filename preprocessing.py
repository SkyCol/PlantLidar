import open3d as o3d
import numpy as np
from .utils import get_angle_vector,get_rotationMatrix_from_vectors

######################Pre-processing######################
def crop_geometry(pcd):
    # Crop pointcloud to get region of interest(ROI) , as well as remove noise points
    print("按键 K 进入裁剪模式")
    print("裁剪点云:《鼠标左键》框选点云，《ctrl+鼠标左键》连线形成一个多边形区域")
    print("按键 C 结束裁剪并保存点云")
    print("按键 Fs 恢复自由查看模式")
    o3d.visualization.draw_geometries_with_editing([pcd])

def downsample(pcd,sample_size,method = "uniform"):
    """
    downsample pointcloud
    Parameters:
        pcd -- open3d pointcloud object
        method -- uniform downsample("uniform") or voxel downsample("voxel") 
        save_path -- can give a path to save the downsampled pointcloud
    Returns:
        pcd -- downsampled pcd
    """
    if method == "uniform":
        pcd = pcd.uniform_down_sample(every_k_points = sample_size)
    if method == "voxel":
        pcd = pcd.voxel_down_sample(voxel_size = sample_size)
    return pcd

def align_pcd(pcd):
    """
    rotate the pointcloud to align the pointcloud with the bounding-box
    Parameters:
        pcd -- open3d pointcloud object
    Returns:
        pcd_r -- aligned pcd
    """
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                         ransac_n=3,
                                            num_iterations=1000)
    normal = plane_model[0:3]
    z = [0,0,1] 
    included_angle = get_angle_vector(normal,z)
    if included_angle > np.pi/2:
        normal=-normal

    R = get_rotationMatrix_from_vectors(normal,z)
    T = np.eye(4)
    T[:3,:3] = R
    pcd_r = pcd.transform(T)

    return pcd_r

def statistical_filtering(pcd,nb_neighbors=5,std_ratio=1):
    """
    Parameters:
        nb_neighbors -- the k nearest points  
        std_ratio -- Threshold based on standard deviation, the smaller the threshold, the more filtering points
    Returns:
        pcd -- the pcd after remove outlier
    """
    cl,ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors,std_ratio=std_ratio)
    pcd = pcd.select_by_index(ind)
    return pcd

def radius_filtering(pcd,nb_points,radius):
    """
    Parameters:
        nb_points -- the threshould of the points num in every sphere 
        radius -- the radius of each sphere
    Returns:
        pcd -- the pcd after remove outlier
    """
    cl,ind = pcd.remove_radius_outlier(nb_points,radius=1)
    pcd = pcd.select_by_index(ind)
    return pcd