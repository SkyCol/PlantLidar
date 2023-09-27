######################Processing######################

import CSF
import numpy as np
import pandas as pd
import open3d as o3d
import tqdm
import math
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy import spatial

import ctypes # for C++
from numpy.ctypeslib import ndpointer # for C++
from .utils import *
from .io import savePCD
import random

current_dir = os.path.dirname(os.path.abspath(__file__))

def classify_ground(pcd,bSloopSmooth=False,cloth_resolution=0.1,rigidness=3,time_step=0.65,class_threshold=0.03,interations=500,plot = False):
    """
    classify ground from pointcloud using csf algorithm
    Parameters:
        pcd -- open3d pointcloud object
        plot -- if True , plot the ground pointcloud
    Returns:
        ground_pcd -- ground pointcloud
        non_ground_pcd -- not ground pointcloud
    """
    xyz = np.asarray(pcd.points)
    csf = CSF.CSF()
    csf.params.bSloopSmooth = bSloopSmooth
    csf.params.cloth_resolution = cloth_resolution
    csf.params.rigidness = rigidness
    csf.params.time_step = time_step
    csf.params.class_threshold = class_threshold
    csf.params.interations = interations
    csf.setPointCloud(xyz)
    ground = CSF.VecInt()                             
    non_ground = CSF.VecInt()    

    csf.do_filtering(ground, non_ground)         

    ground_pcd = pcd.select_by_index(ground)         
    non_ground_pcd = pcd.select_by_index(non_ground)  

    if plot==True:
        o3d.visualization.draw_geometries([ground_pcd])

    return ground_pcd,non_ground_pcd

def RANSAC_classify_ground(pcd):
    """
    Classify ground from pointcloud using RANSAC method
    first : divide the point cloud is into res*res small areas
    second: using RANSAC to fit a plane as the ground for each small area
    last  : merge all the grounds as a whole ground pointcloud
    
    Parameters:
        pcd -- open3d pointcloud object
    Returns:
        ground_pcd -- ground pointcloud
        non_ground_pcd -- not ground pointcloud
    """
    res = 10
    xyz = np.asarray(pcd.points)
    xmin,ymin,zmin = np.min(xyz,axis=0) 
    xmax,ymax,zmax = np.max(xyz,axis=0)
    nrow = math.ceil((xmax-xmin)/res) 
    ncol = math.ceil((ymax-ymin)/res) 
    block_num =  (nrow+1)*(ncol+1) 
    queue = [[] for i in range(block_num)]
    if (xmax-xmin)*(ymax-ymin) > res**2:
        for i in range(xyz.shape[0]):
            current_row = round((xyz[i][0] - xmin) / res)
            current_col = round((xyz[i][1]-ymin) / res)
            current_block = current_row*ncol + current_col
            queue[current_block].append(i)
    
    ground_pcd  = o3d.geometry.PointCloud()
    non_ground_pcd = o3d.geometry.PointCloud()
    for indexs in queue:
        if indexs!=[]:
            current_pcd = pcd.select_by_index(indexs)
            if np.asarray(current_pcd.points).shape[0]<3:
                continue
            plane_model, inliers = current_pcd.segment_plane(distance_threshold=0.1,
                                                            ransac_n=3,
                                                            num_iterations=1000)
            current_non_ground = current_pcd.select_by_index(inliers, invert=True)
            current_ground = current_pcd.select_by_index(inliers)
            non_ground_pcd += current_non_ground
            ground_pcd += current_ground
    
    return ground_pcd,non_ground_pcd
    

def compute_dtm(ground_pcd,res):
    """
    Compute dtm(digital terrian model) on the XoY plane
    Parameters:
        ground_pcd -- ground pointcloud
        res -- size of each pixel to be created (m)
    Returns:
        ground_raster -- digital terrian model on the XoY plane
    """
    ground_xyz = np.asarray(ground_pcd.points)
    xmin,ymin,zmin = np.min(ground_xyz,axis=0) 
    xmax,ymax,zmax = np.max(ground_xyz,axis=0) 
    nrow = round((xmax-xmin)/res)
    ncol = round((ymax-ymin)/res)
    ground_raster = np.zeros((nrow+1,ncol+1))

    points = np.asarray(ground_pcd.points)
    point2d = np.c_[points[:, 0], points[:, 1]]
    tri = spatial.Delaunay(point2d)
    tri_vertexes = tri.simplices


    temp = 0 # memory last value
    for i in tqdm.trange(nrow+1,desc="Nomalize height"): 
        for j in range(ncol+1):
            middle_x = i*res+xmin+res/2
            middle_y = j*res+ymin+res/2
            ntri = tri.find_simplex([(middle_x,middle_y)])
            if ntri[0]!=-1:
                vertex = tri_vertexes[ntri[0]]
                pointa = ground_xyz[vertex[0]]
                pointb = ground_xyz[vertex[1]]
                pointc = ground_xyz[vertex[2]]
                A,B,C,D = compute_plane(pointa,pointb,pointc) 
                ground_raster[i,j] = -(A*middle_x+B*middle_y+D)/C
                temp = ground_raster[i,j]
                
            elif ntri[0] == -1:
                ground_raster[i,j]=temp
    dtm = raster(ground_raster,res,[xmin,xmax,ymin,ymax])

    return dtm

def compute_slope(dtm):
    """
    Compute dsm(digital slope model) on the XoY plane
    Parameters:
        dtm(digital terrian model)
    Returns:
        dsm -- digital slope model on the XoY plane
    """

    xmin,xmax,ymin,ymax = dtm.xmin,dtm.xmax,dtm.ymin,dtm.ymax
    res = dtm.res
    rast = dtm.rast
    horizontal_pixel_spacing = 1.0 
    vertical_pixel_spacing = 1.0    

    dx, dy = np.gradient(rast, horizontal_pixel_spacing, vertical_pixel_spacing)

    slope_percent = np.arctan(np.sqrt(dx ** 2 + dy ** 2)) * 100.0
    # slope_percent = np.arctan(np.sqrt(dx ** 2 + dy ** 2))
    dsm = raster(slope_percent,res,[xmin,xmax,ymin,ymax])

    return dsm





def plot_dtm3d(ground_pcd,algorithm="tin"):
    """
    Compute and visualiza 3d dtm(3d digital terrian model), this need mayavi package!
    Parameters:
        ground_pcd -- ground pointcloud
    Returns:
        None
    """

    from mayavi import mlab
    if algorithm=="tin":
        points = np.asarray(ground_pcd.points)
        point2d = np.c_[points[:, 0], points[:, 1]]
        tri = spatial.Delaunay(point2d)
        mlab.triangular_mesh(points[:,0],points[:,1],points[:,2],tri.simplices)
        mlab.show()

def normalize_height(pcd,ground_pcd,res=0.5,level="dtm"):
    """
    Normalize pointcloud using ground pointcloud got from classify_ground methods
    This is an important step before any processing
    Parameters:
        pcd -- opne3d pointcloud object
        ground_pcd --ground pointcloud
        res -- size of each pixel to be created (m)
        level -- if "dtm" , using dtm to reprensent ground and normalize height(fast)
                 if "pointcloud" , using pointcloud to represent ground and normalize height(slow)
    Rerurns:
        pcd -- height normalized pcd
    """
    # using tin algorithm to compute ground and nomalize height
    xyz = np.asarray(pcd.points)
    ground_xyz = np.asarray(ground_pcd.points)
    xmin,ymin,zmin = np.min(xyz,axis=0) 
    xmax,ymax,zmax = np.max(xyz,axis=0) 
    nrow = round((xmax-xmin)/res)
    ncol = round((ymax-ymin)/res)
    ground_raster = np.zeros((nrow+1,ncol+1))

    points = np.asarray(ground_pcd.points)
    point2d = np.c_[points[:, 0], points[:, 1]]
    tri = spatial.Delaunay(point2d)
    tri_vertexes = tri.simplices

    if level=="dtm": ##  1-----dtm level

        temp = 0 # memory last value
        for i in tqdm.trange(nrow+1,desc="Nomalize height"): 
            for j in range(ncol+1):
                middle_x = i*res+xmin+res/2
                middle_y = j*res+ymin+res/2
                ntri = tri.find_simplex([(middle_x,middle_y)])
                if ntri[0]!=-1:
                    vertex = tri_vertexes[ntri[0]]
                    pointa = ground_xyz[vertex[0]]
                    pointb = ground_xyz[vertex[1]]
                    pointc = ground_xyz[vertex[2]]
                    A,B,C,D = compute_plane(pointa,pointb,pointc) 
                    ground_raster[i,j] = -(A*middle_x+B*middle_y+D)/C
                    temp = ground_raster[i,j]
                    
                elif ntri[0] == -1:
                    ground_raster[i,j]=temp
        for i in range(xyz.shape[0]): 
            current_x = xyz[i][0]
            current_y = xyz[i][1]
            current_row = round((current_x-xmin)/res)
            current_col = round((current_y-ymin)/res)
            xyz[i][2] -= ground_raster[current_row,current_col]
        
    if level == "pointcloud" : ## 2-----pointcloud level
        temp = 0 # memory last value
        for i in tqdm.trange(xyz.shape[0],desc="Nomalize height"): 
            current_x = xyz[i][0]
            current_y = xyz[i][1]
            ntri = tri.find_simplex([(current_x,current_y)])
            if ntri[0]!=-1:
                vertex = tri_vertexes[ntri[0]]
                pointa =  ground_xyz[vertex[0]]
                pointb = ground_xyz[vertex[1]]
                pointc = ground_xyz[vertex[2]]
                A,B,C,D = compute_plane(pointa,pointb,pointc)
                xyz[i][2] -= -(A*current_x+B*current_y+D)/C
                temp = -(A*current_x+B*current_y+D)/C
            elif ntri[0] == -1:
                xyz[i][2] -= temp

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd

def compute_chm(pcd,res=0.5):
    """
    Compute chm(canopy height model) on the XoY plane
    Parameters:
        pcd -- normalized pointcloud
        res -- size of each pixel to be created (m)
    Returns:
        chm -- canopy height model on the XoY plane
    """
    xyz = np.asarray(pcd.points)
    xmin,ymin,zmin = np.min(xyz,axis=0) 
    xmax,ymax,zmax = np.max(xyz,axis=0) 
    ncol = round((xmax-xmin)/res)
    nrow = round((ymax-ymin)/res)
    chm_raster = np.zeros((nrow+1,ncol+1))
    for i in tqdm.trange(xyz.shape[0],desc="Compute chm"): # chm
        current_x = xyz[i][0]
        current_y = xyz[i][1]
        current_col = round((current_x-xmin)/res)
        current_row = round((current_y-ymin)/res)
        chm_raster[current_row,current_col] = max(xyz[i][2],chm_raster[current_row,current_col])
    chm_raster = raster(chm_raster,res,[xmin,xmax,ymin,ymax])
    return chm_raster

def search_treetops_on_pcd(pcd,radius = 2.5,plot=False,save_path =""):
    """
    Get tree tops location in the pointcloud at 3D pointcloud level
    Parameters:
        pcd -- open3d pointcloud object
        radius -- the sphere radius to search the top
        plot -- if true plot the treetops on the pointcloud
        save_path -- the path to save the treetops (csv file)
    Returns:
        tree_tops -- the tree-top points(open3d pointcloud object)
    """
    xyz = np.asarray(pcd.points)
    tree_tops = []
    for i in tqdm.trange(xyz.shape[0],desc="Search treetops"):
        if xyz[i,2]>2:
            current_point = np.expand_dims(xyz[i],axis=0)
            distance_array = compute_distance_array(current_point,xyz).squeeze(0)
            points_index = np.where(distance_array<=radius)
            points = xyz[points_index]
            if  not np.any(points[:,2]>xyz[i,2]):
                tree_tops.append(i)
    tree_tops = pcd.select_by_index(tree_tops)

    if plot==True:
        sphere_list = []
        tops = np.asarray(tree_tops.points)
        for i in range(tops.shape[0]):
            mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0,      
                                                        resolution=15)
            mesh_sphere.compute_vertex_normals()
            mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
            mesh_sphere.translate(tops[i],relative = False)
            sphere_list.append(mesh_sphere)
        pcd = paint_color_by_height(pcd)
        o3d.visualization.draw_geometries([pcd]+sphere_list)

    if not save_path=="":
        data = pd.DataFrame(np.asarray(tree_tops.points),columns=['x','y','z'])
        data.to_csv(save_path,index=False,header=True)
        
    return tree_tops

def search_treetops_on_chm(chm,radius = 2.5,plot = False,save_path =""):
    """
    Get tree tops location in the pointcloud at 2D chm level
    Parameters:
        chm -- a raster object of canopy height model (chm)
        radius -- the sphere radius to search the top
        plot -- if true plot the treetops on the chm
        save_path -- the path to save the treetops (csv file)
    Returns:
        tree_tops -- treetops map , the pixel of treetop equals to the id
    """
    res = chm.res
    rast = chm.rast
    tree_tops = []
    treetops = np.zeros((rast.shape[0],rast.shape[1]))
    id = 1

    # To C++
    res = float(res)
    radius = float(radius)
    so_file_path = os.path.join(current_dir, 'Cplus_methods','Local_maximum.so')
    lib = ctypes.cdll.LoadLibrary( so_file_path)
    lib.c_lmf_bfs.argtypes = (ctypes.c_int,ctypes.c_int,ctypes.c_char_p \
                                ,ctypes.c_int,ctypes.c_int,ctypes.c_float,ctypes.c_float)
    dataptr = rast.ctypes.data_as(ctypes.c_char_p)

    rows,columns = rast.shape[0],rast.shape[1] 

    for i in tqdm.trange(rast.shape[0],desc="Search treetops"):
        for j in range(rast.shape[1]):
            # if rast[i,j] > 2:
            if rast[i,j] > 1:
                # local_maximum = bfs_lmf(np.array([i,j]),rast,res=res,radius=radius) # py version
                local_maximum = lib.c_lmf_bfs(i,j,dataptr,rows,columns,res,radius) # c++ version
                if local_maximum==1:
                    y = j/rast.shape[1]
                    x = i/rast.shape[0]
                    treetops[i,j] = id
                    id+=1
                    tree_tops.append([chm.xmin+y*(chm.xmax-chm.xmin),chm.ymin+x*(chm.ymax-chm.ymin),rast[i,j]])
    if plot == True:
        fig,ax = plt.subplots(1)
        fig.set_dpi(200)
        plt.title("tree tops on chm")
        ax.set_aspect('equal')
        ax.imshow(rast,cmap='rainbow',extent=(chm.xmin,chm.xmax,chm.ymin,chm.ymax),origin="lower")
        for i in tree_tops:
            x = i[0]
            y = i[1]
            circle = Circle((x,y),radius = 1,color = 'red')
            ax.add_patch(circle)
        plt.show()

    if not save_path=="":
        data = pd.DataFrame(np.array(tree_tops),columns=['x','y','z'])
        data.to_csv(save_path,index=False,header=True)

    return np.array(treetops)

def dalponte2016(chm,treetops,th_tree=2,th_seed=0.45,th_crown=0.55,max_crown = 10):
    """
    Segment trees using dalponte2016 algorithm
    Parameters:
        chm -- a raster object of canopy height model (chm)
        treetops -- treetops map , the pixel of treetop equals to the id
        th_tree -- threshold that the pixel can be a tree
        th_seed -- a pixel is added to a region if the height is higher than (the tree height)*(this value)
        th_crown -- a pixel is added to a region if the height is higher than (the current mean height of the region)*(the value)\
        max_crown -- max diameter of a tree
    Returns:
        crowns -- the map of tree crowns, the pixel of crowns is equal to the id
    """
    grown = True
    expend = True
    chm_rast = chm.rast
    nrow = chm_rast.shape[0]
    ncol = chm_rast.shape[1]

    neighbours = []

    region = treetops.copy()
    regiontemp = treetops.copy()

    seeds = {} 
    sum_height = {}
    npixel = {}

    for i in range(nrow):
        for j in range(ncol):
            if treetops[i][j] !=0:
                seeds[treetops[i][j]] = Point(i,j,treetops[i][j])
                sum_height[treetops[i][j]] = chm_rast[i][j]
                npixel[treetops[i][j]] = 1

    while(grown):
        grown = False
        for i in range(1,nrow-1):
            for j in range(1,ncol-1):
                if region[i][j]!=0:
                    id = region[i][j]

                    seed = seeds[id]
                    hSeed = chm_rast[seed.x][seed.y]
                    mean_hCrown = sum_height[id]/npixel[id]

                    neighbours = []

                    neighbours.append (Point(i-1,j,chm_rast[i-1][j]))
                    neighbours.append (Point(i,j-1,chm_rast[i][j-1]))
                    neighbours.append(Point(i,j+1,chm_rast[i][j+1]))
                    neighbours.append(Point(i+1,j,chm_rast[i+1][j]))

                    for k in range(len(neighbours)):
                        px = neighbours[k]
                        if px.z>th_tree:
                            expend = px.z>hSeed*th_seed and\
                            px.z > mean_hCrown*th_crown and\
                            px.z <= hSeed+hSeed*0.05 and\
                            abs(seed.x-px.x)<max_crown and\
                            abs(seed.y-px.y) < max_crown and\
                            region[px.x][px.y] == 0

                            if(expend):
                                regiontemp[px.x][px.y] = region[i][j]
                                npixel[id]+=1
                                sum_height[id]+=chm_rast[px.x][px.y]
                                grown = True
        region = regiontemp
    crowns = raster(region.astype(int),chm.res,[chm.xmin,chm.xmax,chm.ymin,chm.ymax])
    return crowns

def extract_trees(pcd,crowns,save_path=""):
    """
    Extract single tree pointcloud in the pointcloud map
    Parameters:
        pcd -- normalized pointcloud
        crowns -- the map of tree crowns Got from dalponte2016 algorithm
    Returns:
        max_id -- max index of the single tree pointcloud(count number of trees)
        tree_pcds --single tree pointcloud
    """

    xyz = np.asarray(pcd.points)
    res = crowns.res
    rast = crowns.rast
    max_id = np.max(rast)
    tree_clouds = [[] for _ in range(max_id + 1)]

    for i in tqdm.trange(rast.shape[0],desc="Extract trees"):
        for j in range(rast.shape[1]):
            id = rast[i,j]
            if id > 0:
                y = j/rast.shape[1]
                x = i/rast.shape[0]
                current_xmin = crowns.xmin+y*(crowns.xmax-crowns.xmin)
                current_ymin = crowns.ymin+x*(crowns.ymax-crowns.ymin)
                current_xmax = crowns.xmin+y*(crowns.xmax-crowns.xmin)+res
                current_ymax = crowns.ymin+x*(crowns.ymax-crowns.ymin)+res
                tree_points = xyz[(xyz[:, 0] >= current_xmin) & (xyz[:, 0] <= current_xmax) & (xyz[:, 1] >= current_ymin) & (xyz[:, 1] <= current_ymax)
                                  &  (xyz[:, 2] >= 0)] # z>0.1 (to filter out ground point) 

                if tree_clouds[id] !=[]:
                    tree_clouds[id] = np.concatenate((np.array(tree_points),tree_clouds[id]), axis=0)
                else:
                    tree_clouds[id] = np.array(tree_points)

    tree_pcds = [[] for _ in range(max_id + 1)]
    for i in range(1,max_id):
        tree= o3d.geometry.PointCloud()
        tree.points = o3d.utility.Vector3dVector(tree_clouds[i])
        tree_pcds[i]=tree
        if save_path!="":
            savePCD(tree,os.path.join(save_path,"tree_"+str(i)+'.pcd'))        

    return max_id,tree_pcds


def compute_density_map(pcd,res=0.5):
    """
    Compute density map on the XoY plane
    Parameters:
        pcd -- normalized pointcloud
    Returns:
        density_map -- density map of pointcloud on the XoY plane
    """

    xyz = np.asarray(pcd.points)
    xmin,ymin,zmin = np.min(xyz,axis=0) 
    xmax,ymax,zmax = np.max(xyz,axis=0) 
    ncol = round((xmax-xmin)/res)
    nrow = round((ymax-ymin)/res)
    density_map = np.zeros((nrow+1,ncol+1))
    for i in tqdm.trange(xyz.shape[0],desc="Computing density map"): # chm
        current_x = xyz[i][0]
        current_y = xyz[i][1]
        current_col = round((current_x-xmin)/res)
        current_row = round((current_y-ymin)/res)
        if xyz[i,2] > 0: # To remove ground pointcloud
            density_map[current_row,current_col] += 1
    density_map = raster(density_map,res,[xmin,xmax,ymin,ymax])
    return density_map

    


# def compute_tree_density(chm,crowns):
#     """
#     Parameters:
#         chm -- a raster object of canopy height model (chm)
#         crowns -- the map of tree crowns, the pixel of crowns is equal to the id
#     Returns:
#         tree_density -- 0-1, the proportion of the tree area
#     """
#     chm_rast = chm.rast
#     crown_rast = crowns.rast
#     nrow = chm_rast.shape[0]
#     ncol = chm_rast.shape[1]
#     area = len(chm_rast[chm_rast>0])
#     tree_area = len(crown_rast[crown_rast>0])
#     tree_density = tree_area/area
#     return tree_density

def compute_virtual_volumn(non_gound_pcd,res=0.1):
    """
    Parameters:
        non_ground_pcd -- not ground pointcloud
        res -- size of each pixel to be created (m)
    Returns:
        virtual_volumn -- virtual volumn(relative height x res**2)
    """
    xyz = np.asarray(non_gound_pcd.points)
    xmin,ymin,zmin = np.min(xyz,axis=0) 
    xmax,ymax,zmax = np.max(xyz,axis=0) 
    ncol = round((xmax-xmin)/res)
    nrow = round((ymax-ymin)/res)
    top_raster = np.zeros((nrow+1,ncol+1))
    bottom_raster = np.zeros((nrow+1,ncol+1))
    for i in tqdm.trange(xyz.shape[0],desc="Compute virtual volumn"): # chm
        current_x = xyz[i][0]
        current_y = xyz[i][1]
        current_col = round((current_x-xmin)/res)
        current_row = round((current_y-ymin)/res)
        top_raster[current_row,current_col] = max(xyz[i][2],top_raster[current_row,current_col])
        bottom_raster[current_row,current_col] = min(xyz[i][2],bottom_raster[current_row,current_col])
    relative_height = top_raster - bottom_raster
    virtual_volumn = np.sum(relative_height*res*res)

    return virtual_volumn

def compute_crown_area(crowns,treetops,save_path=""):
    """
    Parameters:
        chm -- a raster object of canopy height model (chm)
        crowns -- the map of tree crowns, the pixel of crowns is equal to the id
    Returns:
        tree_density -- 0-1, the proportion of the tree area
    """
    tree_biomass = []
    rast = crowns.rast 
    for i in tqdm.trange(rast.shape[0],desc="Compute crown area"):
        for j in range(rast.shape[1]):
            current_id = rast[i,j]
            y = j/rast.shape[1]
            x = i/rast.shape[0]
            if treetops[i,j]:
                current_tree_area = len(rast[rast==current_id])*crowns.res*crowns.res
                tree_biomass.append([crowns.xmin+y*(crowns.xmax-crowns.xmin),crowns.ymin+x*(crowns.ymax-crowns.ymin),current_tree_area])
    if not save_path=="":
        data = pd.DataFrame(np.array(tree_biomass),columns=['x','y','CrownArea'])
        data.to_csv(save_path,index=False,header=True)

    return len(rast[rast>0])*crowns.res*crowns.res 

def compute_biomass_wheat(pcd,algorithm='3DVI'):
    """
    This is a method of computing above-ground-biomass based on 3DVI and 3DPI
    Parameters:
        pcd -- open3d pointcloud object after nomorlizing
    Returns:
        biomass_wheat -- the above-ground biomass of given wheat pointcloud
    """

    if algorithm == '3DVI':
        res=0.13
        xyz = np.asarray(pcd.points)
        xmin,ymin,zmin = np.min(xyz,axis=0) 
        xmax,ymax,zmax = np.max(xyz,axis=0) 
        n_voxel = math.ceil(zmax-zmin/res)
        nrow = round((xmax-xmin)/res) + 1
        ncol = round((ymax-ymin)/res) + 1

        min_points_per_voxel = 10
        z_threshold = 0.1

        voxel_grid = np.zeros((nrow + 1, ncol + 1, n_voxel + 1))

        row_indices = ((xyz[:, 0] - xmin) / res).astype(int)
        col_indices = ((xyz[:, 1] - ymin) / res).astype(int)
        voxel_indices = ((xyz[:, 2] - zmin) / res).astype(int)

        
        voxel_indices = np.clip(voxel_indices, 0, n_voxel)

        for i in range(len(row_indices)):
            voxel_grid[row_indices[i], col_indices[i], voxel_indices[i]] += 1

        filtered_voxel_grid = np.where(voxel_grid >= min_points_per_voxel, voxel_grid, 0)
        filtered_voxel_grid[:, :, :int(z_threshold / res)] = 0

        num_voxels_with_points = np.sum(filtered_voxel_grid > 0)

        # compute 3D Voxel Index(3DVI)
        voxel_index = num_voxels_with_points / (nrow*ncol)


        biomass_wheat = voxel_index*2.18949-1.26579
    
    elif algorithm == '3DPI':
        z_threshold = 0.1
        k = -1.50

        xyz = np.asarray(pcd.points)
        z_values = xyz[:, 2]
        
        # Filter points based on z_threshold
        indices_to_keep = np.where(z_values >= z_threshold)
        xyz = xyz[indices_to_keep]

        total_points = xyz.shape[0]

        z_values = xyz[:, 2]
        min_z = np.min(z_values)
        max_z = np.max(z_values)

        # create layer by 0.01m
        z_layers = np.arange(min_z, max_z + 0.01, 0.01)

        points_per_layer = np.zeros(len(z_layers), dtype=int)

        for z in z_values:
            if z >= z_threshold:  # Only count points above or equal to z_threshold
                layer_index = int((z - min_z) / 0.01)
                points_per_layer[layer_index] += 1

        cumulative_points = np.cumsum(points_per_layer)
        ThreeDPI = 0 

        for i in range(len(points_per_layer)):
            ThreeDPI += (points_per_layer[i]/total_points) * math.exp(k*cumulative_points[i]/total_points)

        biomass_wheat = 6.97721 * ThreeDPI + 0.69980

    return biomass_wheat
    

def compute_dbh_tree(pcd):
    """
    This is a method of computing above-ground-biomass based on 3D volumn index
    Parameters:
        pcd -- open3d pointcloud object (single tree pointcloud) after normalizing
    Returns:
        dbh -- the dbh given single tree pointcloud
    """

    xyz = np.asarray(pcd.points)

    desired_height = 1.3
    tolerance = 0.05
    height_mask = np.logical_and(xyz[:, 2] >= desired_height - tolerance, xyz[:, 2] <= desired_height + tolerance)
    points_at_desired_height = xyz[height_mask]

    while len(points_at_desired_height) <= 5 and tolerance <= 0.2:  # Increase the maximum tolerance as needed
        height_mask = np.logical_and(xyz[:, 2] >= desired_height - tolerance, xyz[:, 2] <= desired_height + tolerance)
        points_at_desired_height = xyz[height_mask]

        if len(points_at_desired_height) <= 5:
            tolerance += 0.05

    if len(points_at_desired_height) <= 5:
        tree_diameter = random.uniform(0.2, 0.5)
        print("too few points")
    else:
        x_distances = points_at_desired_height[:, 0] - np.mean(points_at_desired_height[:, 0])
        y_distances = points_at_desired_height[:, 1] - np.mean(points_at_desired_height[:, 1])
        distances = np.sqrt(x_distances**2 + y_distances**2)
        
        tree_diameter = 2 * np.max(distances)

    if tree_diameter>1.7:
        tree_diameter = random.uniform(0.2, 0.5)

    return tree_diameter

def compute_biomass_tree_multi(pcd,crowns,save_path=""):
    
    xyz = np.asarray(pcd.points)
    res = crowns.res
    rast = crowns.rast
    max_id = np.max(rast)
    tree_clouds = [[] for _ in range(max_id + 1)]

    for i in tqdm.trange(rast.shape[0],desc="Extract trees"):
        for j in range(rast.shape[1]):
            id = rast[i,j]
            if id > 0:
                y = j/rast.shape[1]
                x = i/rast.shape[0]
                current_xmin = crowns.xmin+y*(crowns.xmax-crowns.xmin)
                current_ymin = crowns.ymin+x*(crowns.ymax-crowns.ymin)
                current_xmax = crowns.xmin+y*(crowns.xmax-crowns.xmin)+res
                current_ymax = crowns.ymin+x*(crowns.ymax-crowns.ymin)+res
                tree_points = xyz[(xyz[:, 0] >= current_xmin) & (xyz[:, 0] <= current_xmax) & (xyz[:, 1] >= current_ymin) & (xyz[:, 1] <= current_ymax)
                                  &  (xyz[:, 2] >= 0)] # z>0.1 (to filter out ground point) 

                if tree_clouds[id] !=[]:
                    tree_clouds[id] = np.concatenate((np.array(tree_points),tree_clouds[id]), axis=0)
                else:
                    tree_clouds[id] = np.array(tree_points)

    data = []
    for i in tqdm.trange(len(tree_clouds),desc="Compute dbh"):
        if i>0:
            current_tree_clouds = tree_clouds[i]
            max_z_index = np.argmax(current_tree_clouds[:, 2])
            point_with_max_z = tree_clouds[i][max_z_index]
            x,y,z = point_with_max_z[0],point_with_max_z[1],point_with_max_z[2]
            tree= o3d.geometry.PointCloud()
            tree.points = o3d.utility.Vector3dVector(tree_clouds[i])
            dbh= compute_dbh_tree(tree)
            data.append([i,x,y,z,dbh])

    if not save_path=="":
        data = pd.DataFrame(np.array(data),columns=['id','x','y','z','dbh'])
        data.to_csv(save_path,index=False,header=True)



# def compute_biomass_test_factor(chm,crowns,save_path=""):
#     """
#     Parameters:
#         chm -- a raster object of canopy height model (chm)
#         crowns -- the map of tree crowns, the pixel of crowns is equal to the id
#         save_path -- the path to save the biomass (csv file)
#     Returns:
#         None
#     """
#     W0 = 0 # tree height
#     W1 = 0 
#     W2 = 0
#     W3 = 0
#     W4 = 0
#     biomass = []
#     chm_rast = chm.rast
#     crowns_rast = crowns.rast
#     res = chm.res
#     max_id = np.max(crowns_rast)
    
#     for id in tqdm.trange(max_id-1,desc="Compute tree biomass"):
#         # id:0 -- None tree
#         id += 1

#         coords = np.argwhere(crowns_rast==id)
#         current_crown_indices = np.where(crowns_rast==id)
#         W0 = np.max(chm_rast[current_crown_indices])
#         # print(chm_rast[current_crown_indices])
#         # print(current_treetop_xy)
#         current_treetop_xy = np.where(chm_rast==W0)
#         x = current_treetop_xy[0][(current_treetop_xy[0] >= np.min(current_crown_indices[0]))
#                                                     & (current_treetop_xy[0] <= np.max(current_crown_indices[0]))] 
#         y = current_treetop_xy[1][(current_treetop_xy[1] >= np.min(current_crown_indices[1]))
#                                                     & (current_treetop_xy[1] <= np.max(current_crown_indices[1]))]
#         x,y = x/chm_rast.shape[0],y/chm_rast.shape[1]
#         # x,y = current_treetop_xy[0]/chm_rast.shape[0],current_treetop_xy[1]/chm_rast.shape[1]
#         x,y=x[0],y[0]

#         W1 = (np.max(current_crown_indices[1]) - np.min(current_crown_indices[1]) + 1)*res
#         W2 = (np.max(current_crown_indices[0]) - np.min(current_crown_indices[0]) + 1)*res
#         distance_matrix = euclidean_distance_matrix(coords)
#         max_distance = np.amax(distance_matrix)
#         W3 = max_distance*res
        
#         current_crown = crowns_rast[np.max(current_crown_indices[1]) : np.min(current_crown_indices[1]),
#                                     np.max(current_crown_indices[0]) : np.min(current_crown_indices[0])]
#         empty_pixel = 0
#         total_pixel =  len(coords)

#         # To C++
#         so_file_path = os.path.join(current_dir, 'Cplus_methods','Count_empty_pixel.so')
#         lib = ctypes.cdll.LoadLibrary(so_file_path)
#         lib.count_surrounding_zeros.argtypes = (ctypes.c_char_p,ctypes.c_int,ctypes.c_int)
#         dataptr = current_crown.ctypes.data_as(ctypes.c_char_p)
#         empty_pixel = lib.count_surrounding_zeros(dataptr,current_crown.shape[0],current_crown.shape[1]) # c++ version

#         full_ratio = 1 - empty_pixel/total_pixel 
#         W4 = full_ratio * (W1+W2+W3)/3

#         W = W0*(W1+W2+W3+W4)
    
#         biomass.append([chm.xmin+y*(chm.xmax-chm.xmin),chm.ymin+x*(chm.ymax-chm.ymin),W])

#         # print(W0,W1,W2,W3,W4)
#         # print("........."+str(W))

#     if not save_path=="":
#         data = pd.DataFrame(np.array(biomass),columns=['x','y','biomass'])
#         data.to_csv(save_path,index=False,header=True)