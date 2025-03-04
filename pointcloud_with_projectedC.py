#!/usr/bin/env python3
import sys # imported sys module
from pathlib import Path # imported Path module

import numpy as np # imported numpy
import open3d as o3d # imported module o3d
from genicam.genapi import NodeMap # imported module NodeMap
from harvesters.core import Component2DImage, Harvester # imported module Harvester and Component2DImage from harvesters.core

from photoneo_genicam.components import enable_components, enabled_components
from photoneo_genicam.default_gentl_producer import producer_path
from photoneo_genicam.pointcloud import (calculate_point_cloud_from_projc, create_3d_vector,
                                         pre_fetch_coordinate_maps)
from photoneo_genicam.user_set import load_default_user_set
from photoneo_genicam.visualizer import RealTimePCLRenderer


def main(device_sn: str): # define the main function
    with Harvester() as h: # consider h as Harvester
        h.add_file(str(producer_path), check_existence=True, check_validity=True) # add file to harvester
        h.update() 

        with h.create({"TER-008": device_sn}) as ia: # connect to device
            features: NodeMap = ia.remote_device.node_map

            load_default_user_set(features) #   load default user set

            print("Pre-fetch CoordinateMaps") #   pre-fetch CoordinateMaps
            coordinate_map: np.array = pre_fetch_coordinate_maps(ia) #   pre-fetch CoordinateMaps

            enable_components(features, ["Range"]) #   enable components with Range

            features.Scan3dOutputMode.value = "ProjectedC" #   set Scan3dOutputMode as ProjectedC
            if features.IsMotionCam3D_Val.value: #   if IsMotionCam3D_Val is true
                features.CameraTextureSource.value = "Laser" #   set CameraTextureSource as Laser
            else:
                features.TextureSource.value = "Laser" #   set TextureSource as Laser

            ia.start() #   start the device
            frame_counter = 0 #   consider the frame_counter as 0
            total_fps = 0.0 #   consider the total_fps as 0.0
            pcl_renderer = RealTimePCLRenderer() #   consider the pcl_renderer as RealTimePCLRenderer
            while not pcl_renderer.should_close: #   if should_close is false
                with ia.fetch(timeout=10) as buffer:
                    depth_map: Component2DImage = buffer.payload.components[0]
                    pcl: np.array = calculate_point_cloud_from_projc(
                        depth_map.data.copy(), coordinate_map
                    )

                    points: o3d.utility.Vector3dVector = create_3d_vector(pcl)
                    if frame_counter == 0: #   if frame_counter is 0
                        point_cloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud(
                            points=points
                        )
                        pcl_renderer.vis.add_geometry(point_cloud) #   add geometry
                    else:
                        point_cloud.points.clear()#   clear points
                        point_cloud.points.extend(points) #   extend points
                        pcl_renderer.vis.update_geometry(point_cloud)
                    frame_counter += 1 #   increment frame_counter
                    total_fps += ia.statistics.fps #   increment total_fps

                    pcl_renderer.vis.poll_events() #   poll events
                    pcl_renderer.vis.update_renderer() #   update renderer

                    print(f"Avg FPS: {round(total_fps / frame_counter, 2)}", end="\r") #   print avg fps

            pcl_renderer.vis.destroy_window() #   destroy window


if __name__ == "__main__": # checking if the name of the file is main
    try:
        device_id = sys.argv[1] #   consider the device_id as sys.argv[1]
    except IndexError: # exception IndexError happens when there is no argument
        print("Error: no device given, please run it with the device serial number as argument:")
        print(f"    {Path(__file__).name} <device serial>")
        sys.exit(1)
    main(device_id) # calling the main function having device_id
