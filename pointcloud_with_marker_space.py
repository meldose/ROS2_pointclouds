#!/usr/bin/env python3
import sys # imported sys module
from pathlib import Path # imported Path module

import numpy as np # imported numpy as np
import open3d as o3d # imported module o3d
from genicam.genapi import NodeMap # imported module NodeMap
from harvesters.core import Component2DImage, Harvester # imported module Harvester and Component2DImage from harvesters.core

from photoneo_genicam.chunks import get_transformation_matrix_from_chunk, parse_chunk_selector
from photoneo_genicam.components import enable_components, enabled_components
from photoneo_genicam.default_gentl_producer import producer_path
from photoneo_genicam.features import enable_software_trigger
from photoneo_genicam.pointcloud import create_3d_vector, map_texture
from photoneo_genicam.user_set import load_default_user_set
from photoneo_genicam.visualizer import render_static


def main(device_sn: str): # define the main function
    with Harvester() as h: # consider h as Harvester
        np.set_printoptions(suppress=True) # set print options as suppress true
        h.add_file(str(producer_path), check_existence=True, check_validity=True) # add file to harvester
        h.update() # update harvester
        

        with h.create({"serial_number": device_sn}) as ia: # connect to device with device_sn
            features: NodeMap = ia.remote_device.node_map

            load_default_user_set(features) # load default user set
            enable_software_trigger(features) # enable software trigger
            enable_components(features, ["Intensity", "Range"]) # enable components with Intensity and Range

            features.Scan3dOutputMode.value = "CalibratedABC_Grid" # set Scan3dOutputMode as CalibratedABC_Grid
            features.RecognizeMarkers.value = True # set RecognizeMarkers as True
            features.CoordinateSpace.value = "MarkerSpace" # set CoordinateSpace as MarkerSpace

            features.ChunkModeActive.value = True # set ChunkModeActive as True
            features.ChunkSelector.value = "CurrentCameraToCoordinateSpaceTransformation" # set ChunkSelector as CurrentCameraToCoordinateSpaceTransformation
            features.ChunkEnable.value = True # set ChunkEnable as True

            ia.start() # start the device
            features.TriggerSoftware.execute() # execute the software trigger

            # If no marker is recognized, the fetch will time out.
            with ia.fetch(timeout=10) as buffer: # fetch the buffer with timeout as 10
                components = dict(zip(enabled_components(features), buffer.payload.components))
                intensity_component: Component2DImage = components["Intensity"] # consider the intensity_component as Component2DImage with Intensity
                point_cloud_raw: Component2DImage = components["Range"] # consider the point_cloud_raw as Component2DImage with Range

                chunk_name: str = "CurrentCameraToCoordinateSpaceTransformation" # chunk_name as CurrentCameraToCoordinateSpaceTransformation
                parsed_chunk_data: dict = parse_chunk_selector(
                    features, chunk_feature_name=chunk_name
                )
                transformation_matrix: np.ndarray = get_transformation_matrix_from_chunk(
                    parsed_chunk_data
                )
                print(transformation_matrix) # print the transformation_matrix

                point_cloud = o3d.geometry.PointCloud() # consider the point_cloud as o3d.geometry.PointCloud
                point_cloud.points = create_3d_vector(point_cloud_raw.data) # consider the point_cloud.points as create_3d_vector
                point_cloud.colors = map_texture(intensity_component) # consider the point_cloud.colors as map_texture
                point_cloud.transform(transformation_matrix)
                render_static([point_cloud]) # render the point_cloud


if __name__ == "__main__": # checking if the name of the file is main
    try:
        device_id = sys.argv[1]
    except IndexError: # exception IndexError happens when there is no argument
        print("Error: no device given, please run it with the device serial number as argument:") # print error message
        print(f"    {Path(__file__).name} <device serial>")
        sys.exit(1)
    main(device_id) # calling the main function having device_id