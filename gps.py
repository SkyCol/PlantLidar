import numpy as np
import open3d as o3d

######################GPS######################
def xyz2GPS(pcd,oneGPS,oneXYZ):
    """
    transform relativate coordinate to GPS coordinate
    Parameters:
        pcd -- open3d pointcloud object with relative coordinate
        oneGPS -- one GPS like [5523323,123421] 
        oneXYZ -- one XYZ in pcd like [-12.523569,10.786754]
    Returns:
        pcd_gps -- pcd with gps coordinate
    """
    np.set_printoptions(suppress=True)
    bits = len(str(oneXYZ[0]).split('.')[1]) # decimal digits
    xyz = np.asarray(pcd.points)
    xyz = np.round(xyz,bits)
    # orient_index = np.where((xyz[:,0]== oneXYZ[0])&(xyz[:,1] == oneXYZ[1]))
    xy_offset = xyz[:,0:2] - oneXYZ
    xyz[:,0:2] = xy_offset + oneGPS # GPS
    pcd_gps = o3d.geometry.PointCloud()
    pcd_gps.points = o3d.utility.Vector3dVector(xyz)
    return pcd_gps

def GPS2xyz(pcd):
    """
    transform relativate coordinate to GPS coordinate
    Parameters:
        pcd -- open3d pointcloud object with GPS coordinate
    Returns:
        pcd_relative -- pcd with relative coordinate
    """
    xyz = np.asarray(pcd.points)
    # orient_index = np.where((xyz[:,0]== oneXYZ[0])&(xyz[:,1] == oneXYZ[1]))
    xyz[:,0:2] = xyz[:,0:2] - xyz[0,0:2]
    pcd_relative = o3d.geometry.PointCloud()
    pcd_relative.points =  o3d.utility.Vector3dVector(xyz)
    return pcd_relative