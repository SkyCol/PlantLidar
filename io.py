import open3d as o3d
import os
######################IO######################
def readPCD(filepath):
    # read a pointcloud(pcd,ply,txt,csv)
    pcd_ = o3d.io.read_point_cloud(filepath)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_.points)
    return pcd

def readPCDS(filepath):
    # read pointclouds under the given path together(pcd,ply,txt,csv)
    files = os.listdir(filepath)
    pcd = o3d.geometry.PointCloud()
    for file in files:
        temp = o3d.io.read_point_cloud(file)
        pcd+=temp
    return pcd

def readOBJ(filepath):
    # read an obj file(.obj)
    obj= o3d.io.read_triangle_mesh(filepath)
    return obj


def savePCD(pcd,save_path,GPS=False):
    # save pointcloud in save_path(pcd,ply,txt,csv)
    # if using utm50 GPS coordinate, must save as .ply
    o3d.io.write_point_cloud(save_path,pcd)


def merge_pcds(pcds:list):
    # merge multi pointclouds into one
    pcd = o3d.geometry.PointCloud()
    for cloud in pcds:
        pcd+=cloud
    return pcd

def save_tif(array,save_path):
    # save dtm (Digital Terrian Model) or chm (canopy height model) or dem (Digital Elevation Model) as tif 
    import rasterio
    with rasterio.open(save_path, 'w', driver='GTiff', width=array.shape[1], height=array.shape[0], count=1, dtype=array.dtype) as dst:
        dst.write(array, 1)

