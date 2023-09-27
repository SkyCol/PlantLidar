######################Searching######################
import pandas as pd
import numpy as np
from .utils import compute_distance_array

def search_trees_in_database(position,filepath):
    """
    Parameters:
        position -- 1D array for single search , 2D array for multi search . Represents the coordinate of the tree
        filepath -- file path
    Returns:
        height -- tree height
    """    
    data = pd.read_csv(filepath)
    xy  = np.array(position)
    dim_num = xy.ndim
    if dim_num == 1:
        distances = compute_distance_array([xy],data[["x","y"]])
        index = np.argmin(distances)
        height = data.iloc[index,]["z"]
        return height    
    else:
        distances = compute_distance_array(xy,data[["x","y"]])
        index = np.argmin(distances,axis=1)
        height = data.iloc[index,]["z"]
        return height.values

def search_treeArea_in_database(position,filepath):
    """
    Parameters:
        position -- 1D array for single search , 2D array for multi search . Represents the coordinate of the tree
        filepath -- file path
    Returns:
        treeArea -- tree area
    """    
    data = pd.read_csv(filepath)
    xy  = np.array(position)
    dim_num = xy.ndim
    if dim_num == 1:
        distances = compute_distance_array([xy],data[["x","y"]])
        index = np.argmin(distances)
        treeArea = data.iloc[index,]["CrownArea"]
        return treeArea    
    else:
        distances = compute_distance_array(xy,data[["x","y"]])
        index = np.argmin(distances,axis=1)
        treeArea = data.iloc[index,]["CrownArea"]
        return treeArea.values

def search_treeBiomass_in_database(position,filepath):
    """
    Parameters:
        position -- 1D array for single search , 2D array for multi search . Represents the coordinate of the tree
        filepath -- file path
    Returns:
        treeArea -- tree area
    """    
    data = pd.read_csv(filepath)
    xy  = np.array(position)
    dim_num = xy.ndim
    if dim_num == 1:
        distances = compute_distance_array([xy],data[["x","y"]])
        index = np.argmin(distances)
        treeArea = data.iloc[index,]["biomass"]
        return treeArea    
    else:
        distances = compute_distance_array(xy,data[["x","y"]])
        index = np.argmin(distances,axis=1)
        treeArea = data.iloc[index,]["biomass"]
        return treeArea.values