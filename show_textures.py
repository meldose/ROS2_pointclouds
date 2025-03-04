#!/usr/bin/env python3
import sys # imported sys module
from dataclasses import dataclass # imported dataclass
from pathlib import Path # imported Path
from typing import List # imported List

import cv2 # imported cv2
from genicam.genapi import NodeMap # imported module NodeMap from genicam
from harvesters.core import Component2DImage, Harvester # imported module Harvester and Component2DImage from harvesters.core

from photoneo_genicam.components import enable_components # imported function enable_components from photoneo_genicam.components
from photoneo_genicam.default_gentl_producer import producer_path # imported module producer_path from photoneo_genicam.default_gentl_producer
from photoneo_genicam.user_set import load_default_user_set # imported function load_default_user_set from photoneo_genicam.user_set
from photoneo_genicam.visualizer import TextureImage # imported class TextureImage from photoneo_genicam.visualizer


@dataclass
class TextureSourceConfig: # defined dataclass TextureSourceConfig
    texture_source: str = "Color" # set texture_source = "Color"
    operation_mode: str = "Camera" # set operation_mode = "Camera"
    pixel_format: str = "RGB8" # set pixel_format = "RGB8"
    component: str = "Intensity" # set component = "Intensity"
    camera_space: str = "PrimaryCamera" # set camera_space = "PrimaryCamera"
    name = "" # set name = ""

    def apply_settings(self, features: NodeMap): # defined function apply_settings with two parameters
        is_color: bool = features.IsMotionCam3DColor_Val.value # set is_color = features.IsMotionCam3DColor_Val.value
        is_mc: bool = features.IsMotionCam3D_Val.value # set is_mc = features.IsMotionCam3D_Val.value

        if is_mc: # if is_mc is true
            features.OperationMode.value = self.operation_mode
            if self.operation_mode == "Camera": # if operation_mode is Camera
                features.CameraTextureSource.value = self.texture_source
            else:
                features.TextureSource.value = self.texture_source
        else:
            features.TextureSource.value = self.texture_source

        enable_components(features, [self.component]) # enable_components(features, [self.component])

        if is_color: # if is_color is true
            features.CameraSpace.value = self.camera_space # set features.CameraSpace.value = self.camera_space

        features.PixelFormat.value = self.pixel_format #    set features.PixelFormat.value = self.pixel_format

        self.name = (
            f"TextureSource: {self.texture_source}, "
            f"PixelFormat: {self.pixel_format}, "
            f"Component: {self.component}, "
            f"CameraSpace: {self.camera_space}"
        )


def device_based_configs(features: NodeMap) -> List[TextureSourceConfig]: # defined function device_based_configs with one parameter
    intensity_rgb = TextureSourceConfig(
        component="Intensity", # set component = "Intensity"
        camera_space="PrimaryCamera", # set camera_space = "PrimaryCamera"
        texture_source="Color", #   set texture_source = "Color"
        pixel_format="RGB8", # set pixel_format = "RGB8"
    )
    intensity_mono10 = TextureSourceConfig( # set intensity_mono10 = TextureSourceConfig
        component="Intensity", # set component = "Intensity"
        camera_space="PrimaryCamera", # set camera_space = "PrimaryCamera"
        texture_source="LED", # set texture_source = "LED"
        pixel_format="Mono10", # set pixel_format = "Mono10"
    )
    color_rgb = TextureSourceConfig(
        component="ColorCamera", # set component = "ColorCamera"
        camera_space="PrimaryCamera", # set camera_space = "PrimaryCamera"
        texture_source="Color", # set texture_source = "Color"
        pixel_format="RGB8", # set pixel_format = "RGB8"
    )
    color_mono16 = TextureSourceConfig( # set color_mono16 = TextureSourceConfig
        component="ColorCamera", # set component = "ColorCamera"
        camera_space="PrimaryCamera", # set camera_space = "PrimaryCamera"
        texture_source="Color", # set texture_source = "Color"
        pixel_format="Mono16", # set pixel_format = "Mono16"
    )
    intensity_rgb_camera_space = TextureSourceConfig( # set intensity_rgb_camera_space = TextureSourceConfig
        component="Intensity", # set component = "Intensity"
        camera_space="ColorCamera", # set camera_space = "ColorCamera"
        texture_source="Color", # set texture_source = "Color"
        pixel_format="RGB8", # set pixel_format = "RGB8"
    )
    scanner_default = TextureSourceConfig( # set scanner_default = TextureSourceConfig
        component="Intensity", texture_source="LED", pixel_format="Mono12" # set component = "Intensity", texture_source = "LED", pixel_format = "Mono12"
    )
    alpha_default = TextureSourceConfig( # set alpha_default = TextureSourceConfig
        component="Intensity", texture_source="LED", pixel_format="Mono10" # set component = "Intensity", texture_source = "LED", pixel_format = "Mono10"
    )

    is_color: bool = features.IsMotionCam3DColor_Val.value # set is_color = features.IsMotionCam3DColor_Val.value
    is_mc: bool = features.IsMotionCam3D_Val.value # set is_mc = features.IsMotionCam3D_Val.value
    is_scanner: bool = features.IsPhoXi3DScanner_Val.value # set is_scanner = features.IsPhoXi3DScanner_Val.value
    is_alpha: bool = features.IsAlphaScanner_Val.value # set is_alpha = features.IsAlphaScanner_Val.value
 
    if is_color: # if is_color is true
        print(f"Device type: MotionCam3DColor") # print Device type: MotionCam3DColor
        return [
            intensity_rgb,
            intensity_mono10,
            color_rgb,
            color_mono16,
            intensity_rgb_camera_space,
        ]
    if is_scanner and not is_alpha: # if is_scanner is true and is_alpha is false
        print(f"Device type: PhoXi3DScanner") # print Device type: PhoXi3DScanner
        return [scanner_default]
    if is_alpha and is_scanner: # if is_alpha is true and is_scanner is true
        print(f"Device type: AlphaScanner") # print Device type: AlphaScanner
        return [alpha_default]
    if is_mc: # if is_mc is true
        print(f"Device type: MotionCam3D") # print Device type: MotionCam3D
        return [intensity_mono10] # return intensity_mono10
    else:
        raise Exception("No config defined for current device type") # raise exception


def main(device_sn: str): # defined function main with one parameter
    with Harvester() as h: # consider h as Harvester
        h.add_file(str(producer_path), check_existence=True, check_validity=True) # add file to harvester
        h.update() # update harvester

        images = [] # create an empty list with name images
        with h.create({"serial_number": device_sn}) as ia: # connect to device
            features = ia.remote_device.node_map
            example_options: List[TextureSourceConfig] = device_based_configs(features)
            load_default_user_set(features) # load default user set with example_options

            for setting_combination in example_options: # for setting_combination in example_options
                setting_combination.apply_settings(features)

                ia.start() # start the device
                with ia.fetch(timeout=10) as buffer: # fetch the buffer
                    img: Component2DImage = buffer.payload.components[0]
                    images.append(TextureImage(f"{setting_combination.name}", image=img))
                ia.stop() # stop the device

        if len(images) == 0: # if length of images is 0
            print("No images captured") # print No images captured
            return

        for image in images: # for image in images
            image.show() # show the image

        print("Press ESC to close all windows") # print Press ESC to close all windows
        while True: # infinite loop
            key = cv2.waitKey(1) & 0xFF # it waits for a key press
            if key == 27: # if the key is ESC
                break

        cv2.destroyAllWindows() # clear the windows


if __name__ == "__main__": # checking if the name of the file is main
    try:
        device_id = sys.argv[1]
    except IndexError: # exception IndexError happens when there is no argument
        print("Error: no device given, please run it with the device serial number as argument:")
        print(f"    {Path(__file__).name} <device serial>") #     print the path file with device serial
        sys.exit(1)
    main(device_id) # calling the main function having device_id
