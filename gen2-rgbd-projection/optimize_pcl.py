#!/usr/bin/env python3

"""
Test script for optimizing point clouds.

Some bg info: https://programmer.help/blogs/pcl-preprocessing-filtering.html

"""

import open3d
from pathlib import Path

path_to_current_file = Path(__file__).parent

visualizers = []

def render_point_cloud(name, point_cloud):
    print("––––")
    print(f"Render point cloud \"{name}\"")
    print(point_cloud)

    visualizer = open3d.visualization.Visualizer()
    visualizer.create_window(window_name=name, width=1920, height=1080)
    visualizer.add_geometry(point_cloud)
    origin = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
    visualizer.add_geometry(origin)

    visualizers.append(visualizer)

def run():
    is_active = True
    while is_active:
        for visualizer in visualizers:
            if not visualizer.poll_events():
                is_active = False
                break
            visualizer.update_renderer()

    for visualizer in visualizers:
        visualizer.close()

point_cloud = open3d.io.read_point_cloud(str(
    path_to_current_file.joinpath("desk/point_cloud.pcd")
))

# Flip it, otherwise the pointcloud will be upside down
point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

render_point_cloud("Original", point_cloud)

# no change here
# finite_pcd = point_cloud.remove_non_finite_points()
# render_point_cloud("Finite", finite_pcd)

statistical_outliers_pcd = point_cloud.remove_statistical_outlier(30, 0.1)[0]
render_point_cloud("Statistical Outliers", statistical_outliers_pcd)

downsampled_outliers_pcd = statistical_outliers_pcd.voxel_down_sample(0.01)
render_point_cloud("Downsampled", downsampled_outliers_pcd)

# we might use the passthrough filter from pcl library… using the python wrapper:
# https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/PassThroughFilter.py

# Needs really, really long to compute!
# radius_outliers_pcd = point_cloud.remove_radius_outlier(10, 10)[0]
# render_point_cloud("Radius Outliers", radius_outliers_pcd)

run()
