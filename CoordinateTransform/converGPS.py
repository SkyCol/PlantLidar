'''
WGS84的经纬度 转 UTM的x,y
epsg:泰安市-32650
更多查询 https://www.dmap.co.uk/utmworld.htm
'''
from pyproj import Transformer
from posixpath import split
import pandas as pd
import numpy as np

def WGS2UTM(wgs84_path,utm50_path):
    """
    Parameters:
        wgs84_data -- wgs84 source data(csv:Latitude,Longitude) 
    Returns:
        utm50_path -- utm50 target data(csv:x,y)
    """
    transformer = Transformer.from_crs("epsg:4326", "epsg:32650")
    data=pd.read_csv(wgs84_path)
    lat=data["Latitude"]
    lon=data["Longitude"]

    for i in range(len(lat)):
        lat_str = str(lat[i])
        N = np.float64(lat_str[0:2])
        w = np.float64(lat_str[2:])/60
        lat[i] = N+w

    for i in range(len(lon)):
        lon_str = str(lon[i])
        E = np.float64(lon_str[0:3])
        w = np.float64(lon_str[3:])/60
        lon[i] = E+w

    x, y = transformer.transform(lat, lon)
    pd.DataFrame({"x":x,"y":y}).to_csv(utm50_path,index=False)