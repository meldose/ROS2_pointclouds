#!/usr/bin/env python3
import sys # imported sys module
from pathlib import Path # imported Path module

from genicam.genapi import IEnumeration, NodeMap # imported IEnumeration and NodeMap
from harvesters.core import Harvester # imported module Harvester

from photoneo_genicam.default_gentl_producer import producer_path # imported module producer_path from photoneo_genicam.default_gentl_producer
from photoneo_genicam.user_set import load_default_user_set # imported function load_default_user_set from photoneo_genicam.user_set


def pretty_print_user_set_options(features: NodeMap): # function pretty_print_user_set_options

    def node_exists(s): # defined function node_exists
            features.get_node(s) # get_node
            return True
        except Exception as e:
            return False

    def is_enum(setting_name): # defined function is_enum
        return isinstance(features.get_node(setting_name), IEnumeration)

    print(f"Available user set settings: ") # print statement
    for setting in features.UserSetFeatureSelector.symbolics: # checking the symbolics of the UserSetFeatureSelector
        if node_exists(setting) and is_enum(setting): # checking the symbolics of the UserSetFeatureSelector
            opts = features.get_node(setting).symbolics # assign the opts to the symbolics of the UserSetFeatureSelector
            print(f"{setting} - [{opts}]") # print statement
        else:
            print(f"{setting}")


def main(device_sn: str): # define the main function
    with Harvester() as h: # consider h as Harvester
        h.add_file(str(producer_path), check_existence=True, check_validity=True) # add file to harvester
        h.update() # update harvester

        with h.create({"serial_number": device_sn}) as ia: # connect to device
            features: NodeMap = ia.remote_device.node_map
            load_default_user_set(features) # load default user set

            settings_map = { # set an dictionary with settings_map as key and value
                "CalibrationVolumeOnly": False,
                "TextureSource": "Laser",
                "ExposureTime": 10.24,
                "LEDPower": 2000,
                "ShutterMultiplier": 2,
                "NormalsEstimationRadius": 2,
            }

            print(f"Available user sets: {features.UserSetSelector.symbolics}") # print statement

            print("Changing some settings:")
            for s, v in settings_map.items(): # for s, v in settings_map.items
                print(f"  {s}: {features.get_node(s).value} -> {v}") # print statement
                features.get_node(s).value = v # assign value to features.get_node(s).value = v

            print()
            print(f"Store these changes into UserSet1") # print statement
            features.UserSetSelector.value = "UserSet1" # set the value of UserSetSelector to UserSet1
            features.UserSetSave.execute() # execute the UserSetSave feature
            print(f"OK") # print statement

            print()
            print(f"Load Default profile to restore default setting values") # print statement 
            features.UserSetSelector.value = "Default" # set the value of UserSetSelector to Default
            features.UserSetLoad.execute() # execute the UserSetLoad feature
            print(f"OK") # print statement

            print()
            print(f"Restored settings:") # print statement
            for s, v in settings_map.items(): # for s, v in settings_map.items
                print(f"  {s}: {features.get_node(s).value}") # print statement

            print()
            print("Load UserSet1")
            features.UserSetSelector.value = "UserSet1" # set the value of UserSetSelector to UserSet1
            features.UserSetLoad.execute() # execute the UserSetLoad feature
            print(f"OK")

            print()
            print(f"Current settings (from UserSet1):") # print statement
            for s, v in settings_map.items(): # for s, v in settings_map.items
                print(f"  {s}: {features.get_node(s).value}")


if __name__ == "__main__": # checking if the name of the file is main
    try:
        device_id = sys.argv[1]
    except IndexError: # exception IndexError happens when there is no argument
        print("Error: no device given, please run it with the device serial number as argument:") # print error message
        print(f"    {Path(__file__).name} <device serial>")
        sys.exit(1) # exit the program
    main(device_id) # calling the main function having device_id
