#!/usr/bin/env python3
import sys # imported sys module
from pathlib import Path # imported Path module

import cv2 # imported cv2 module
from genicam.genapi import NodeMap # imported module NodeMap
from harvesters.core import Component2DImage, Harvester # imported module Harvester and Component2DImage from harvesters.core

from photoneo_genicam.components import enable_components # imported function enable_components
from photoneo_genicam.default_gentl_producer import producer_path # imported module producer_path
from photoneo_genicam.features import enable_software_trigger # imported function enable_software_trigger
from photoneo_genicam.user_set import load_default_user_set #   imported function load_default_user_set
from photoneo_genicam.visualizer import process_for_visualisation # imported function process_for_visualisation


def main(device_sn: str): # defined function main having device_sn as an argument
    with Harvester() as h: # consider h as Harvester
        h.add_file(str(producer_path), check_existence=True, check_validity=True) # add file to harvester
        h.update() # update harvester

        with h.create({"serial_number": device_sn}) as ia: # connect to device
            features: NodeMap = ia.remote_device.node_map

            load_default_user_set(features) # load default user set
            enable_software_trigger(features) # enable software trigger

            enable_components(features, ["Range", "Confidence"]) # enable components with Range and Confidence
            features.Scan3dOutputMode.value = "ProjectedC" # set Scan3dOutputMode as ProjectedC

            ia.start() # start the device
            features.TriggerSoftware.execute() # execute the software trigger
            with ia.fetch(timeout=10) as buffer:
                depth: Component2DImage = buffer.payload.components[0]
                confidence: Component2DImage = buffer.payload.components[1]

                dept_img = process_for_visualisation(depth) # assign process_for_visualisation function to dept_img
                cnf_img = process_for_visualisation(confidence) # assign process_for_visualisation function to cnf_img
                cv2.imshow("DepthMap", dept_img) # show the image with depthMap
                cv2.imshow("ConfidenceMap", cnf_img) # show the image with confidenceMap

            while True: # infinite loop
                key = cv2.waitKey(1) & 0xFF # wait for a key press
                if key == 27: # if the key is ESC
                    break

                # Add a check for window closing event
                if cv2.getWindowProperty('DepthMap', cv2.WND_PROP_VISIBLE) < 1: # if the window is not visible with DepthMap
                    break
                if cv2.getWindowProperty('ConfidenceMap', cv2.WND_PROP_VISIBLE) < 1: # if the window is not visible with ConfidenceMap
                    break

            cv2.destroyAllWindows() # clear the windows


if __name__ == "__main__": # checking if the name of the file is main
    try:
        device_id = sys.argv[1]
    except IndexError: # exception IndexError happens when there is no argument
        print("Error: no device given, please run it with the device serial number as argument:")
        print(f"    {Path(__file__).name} <device serial>")
        sys.exit(1)
    main(device_id) #calling the main function having device_id
