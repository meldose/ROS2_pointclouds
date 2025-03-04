#!/usr/bin/env python3
import sys # imported sys module
from pathlib import Path # imported Path module

import open3d as o3d # imported module o3d
from genicam.genapi import NodeMap # imported module NodeMap
from harvesters.core import Component2DImage, Harvester # imported module Harvester and Component2DImage from harvesters.core

from photoneo_genicam.components import enable_components, enabled_components
from photoneo_genicam.default_gentl_producer import producer_path
from photoneo_genicam.features import enable_software_trigger
from photoneo_genicam.pointcloud import create_3d_vector, map_texture
from photoneo_genicam.user_set import load_default_user_set
from photoneo_genicam.visualizer import render_static


def main(device_sn: str): # define the main function
    with Harvester() as h: # consider h as Harvester
        h.add_file(str(producer_path), check_existence=True, check_validity=True) # add file to harvester
        h.update() # update harvester

        with h.create({"serial_number": device_sn}) as ia: # connect to device
            features: NodeMap = ia.remote_device.node_map

            load_default_user_set(features) # load default user set
            enable_software_trigger(features) # enable software trigger

            features.Scan3dOutputMode.value = "CalibratedABC_Grid" # set Scan3dOutputMode as CalibratedABC_Grid
            enable_components(features, ["Intensity", "Range", "Normal"]) # enable components with Intensity, Range and Normal

            ia.start() # start the device
            features.TriggerSoftware.execute() # execute the software trigger
            with ia.fetch(timeout=10) as buffer: # fetch the buffer with timeout as 10
                components = dict(zip(enabled_components(features), buffer.payload.components))
                intensity_component: Component2DImage = components["Intensity"] # consider the intensity_component as Component2DImage with Intensity
                point_cloud_raw: Component2DImage = components["Range"] # consider the point_cloud_raw as Component2DImage with Range
                normal_component: Component2DImage = components["Normal"] # consider the normal_component as Component2DImage with Normal

                point_cloud = o3d.geometry.PointCloud() # create a point cloud
                point_cloud.points = create_3d_vector(point_cloud_raw.data) # set points
                point_cloud.normals = create_3d_vector(normal_component.data) # set normals
                point_cloud.colors = map_texture(intensity_component) # set colors
                render_static([point_cloud]) # render the point cloud


if __name__ == "__main__": # checking if the name of the file is main
    try:
        device_id = sys.argv[1]
    except IndexError: # exception IndexError happens when there is no argument
        print("Error: no device given, please run it with the device serial number as argument:")
        print(f"    {Path(__file__).name} <device serial>")
        sys.exit(1)
    main(device_id) # calling the main function having device_id
