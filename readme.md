# PlantLidar : PointCloud analysis focus on solving 3D phenotypes of plants and remote sensing problems

![seg](asserts/seg.png)



## Installation
```
pip install PlantLidar
```

## To get pointcloud
if using a lidar, build a map from lidar scan through slam algorithm([FAST-LIO](https://github.com/hku-mars/FAST_LIO)) .   
using a camera can also get a pointcloud by inputing multiple view images to [colmap](https://github.com/colmap/colmap) .


## Quick Run
More details please check [PlantLidar-wiki](https://github.com/SkyCol/PlantLidar/wiki/PlantLidar-wiki)
```
import PlantLidar as pl

pcd = pl.io.readPCD("data/example_park.pcd")
pl.plot3D(pcd)

ground_pcd,non_ground_pcd = pl.processing.classify_ground(pcd,cloth_resolution=0.15,interations=500,
                                                          class_threshold = 0.16,plot=False)
# pl.visualization.show_ground_result(ground_pcd,non_ground_pcd)
pcd = pl.processing.normalize_height(pcd,ground_pcd,res=0.3,level="dtm")

density_map = pl.processing.compute_density_map(pcd,res=0.5)
pl.plot2D(density_map,title="density map")

chm = pl.compute_chm(pcd,0.3)
pl.plot2D(chm,title="canopy height model")

treetops = pl.search_treetops_on_chm(chm,2.5)
crowns = pl.dalponte2016(chm,treetops)
pl.plot3D(pcd,crowns,method="segment")
```
