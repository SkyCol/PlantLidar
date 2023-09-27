######################Visualization######################

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from .gps import GPS2xyz

def show_ground_result(ground_pcd,nonground_pcd):

    # ground_pcd = o3d.geometry.PointCloud()
    # ground_pcd.points = o3d.utility.Vector3dVector(ground.points)
    # nonground_pcd = o3d.geometry.PointCloud()
    # nonground_pcd.points = o3d.utility.Vector3dVector(nonground.points)
    # use different color to plot the ground and the nonground
    
    ground_pcd.paint_uniform_color(np.array([0.0,1.0,0.0]))
    nonground_pcd.paint_uniform_color(np.array([1.0,0.0,0.0]))

    o3d.visualization.draw_geometries([ground_pcd,nonground_pcd])

def plot3D(pcd,crown="",method="",GPS=False):

    if GPS==True:
        pcd = GPS2xyz(pcd)

    if method=="":
        # plot pointcloud colored by height
        xyz = np.asarray(pcd.points)
        colors = np.zeros([xyz.shape[0], 3])
        z_max = np.max(xyz[:, 2])
        z_min = np.min(xyz[:, 2])
        delta_c = abs(z_max - z_min) / (255 * 2)
        for j in range(xyz.shape[0]):
            color_n = (xyz[j, 2] - z_min) / delta_c
            if color_n <= 255:
                colors[j, :] = [0, 1 - color_n / 255, 1]
            else:
                colors[j, :] = [(color_n - 255) / 255, 0, 1]
        
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

    if method == "segment":
        # plot pointcloud colored by segmented trees
        res = crown.res
        xmin = crown.xmin
        ymin = crown.ymin
        crown_rast= crown.rast

        xyz = np.asarray(pcd.points)
        colors = np.zeros([xyz.shape[0], 3])
        color_library = np.random.rand(crown_rast.max()+1,3)
        for i in range(xyz.shape[0]):
            current_x = xyz[i][0]
            current_y = xyz[i][1]
            current_col = round((current_x-xmin)/res)
            current_row = round((current_y-ymin)/res)
            if crown_rast[current_row][current_col] !=0:
                colors[i] = color_library[crown_rast[current_row][current_col]]

        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])
    
    if method == "GPS":
        pass
        
def plot2D(ras,title="",method=""):
    # if method=="":
    #     fig = plt.figure(dpi=200)
    #     if title!="":
    #         plt.title(title)
    #     plt.imshow(ras.rast,cmap='rainbow',extent=(ras.xmin,ras.xmax,ras.ymin,ras.ymax),origin="lower")
    #     plt.colorbar()
    #     plt.show()
    if method == "":
        fig = plt.figure(dpi=200)
        if title != "":
            plt.title(title)
        rast= ras.rast
    
    if method == "log":
        fig = plt.figure(dpi=200)
        if title != "":
            plt.title(title)
        # Apply logarithmic transformation to enhance contrast
        rast = np.log1p(ras.rast)  # Applying natural logarithm to the raster values
        
    plt.imshow(rast, cmap='rainbow', extent=(ras.xmin, ras.xmax, ras.ymin, ras.ymax), origin="lower")
    plt.colorbar()
    plt.show()


def plot_by_time (time_list:list,value_list:list):
    """
        time_list -- a list of time
        value_list -- a list of numbers computed in time_list
    """
    plt.xticks(time_list)
    plt.plot(time_list,value_list,marker='o', color = 'pink',markeredgecolor='g',markersize=15, markeredgewidth=3)
    plt.show()